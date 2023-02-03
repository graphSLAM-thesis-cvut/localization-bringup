#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <eigen_conversions/eigen_msg.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <boost/array.hpp>

using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

class OdomSlam
{
private:
    // ROS variables
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_robot_pose_;
    ros::Publisher publisher_robot_optimized_path_;
    ros::Publisher publisher_robot_poses_;
    ros::Time lastInsTime_;

    // ROS topics
    std::string ros_topic_odometry_ = "/camera/odom/sample";
    std::string publisher_robot_optimized_path_topic_ = "path_optimized";
    std::string publisher_robot_poses_topic_ = "robot_poses";

    // GTSAM variables

    // ExpressionFactorGraph is a NonLinearFactorGraph that can
    // *additionally* use expressions as factors. This is useful
    // for us because it allows us to work with BearingRange
    // expressions
    gtsam::NonlinearFactorGraph::shared_ptr graph_;

    gtsam::Values::shared_ptr initial_estimate_;
    gtsam::Values result_;
    gtsam::ISAM2 *isam2_ = nullptr;
    int insertPeriod_ = 5;
    bool debug_ = true;

    // SLAM variables
    int robot_pose_counter_ = -1;

    //
    std::string rviz_ins_frame_ = "camera_odom";

    // pose variables
    gtsam::Pose3 newPose_;
    gtsam::Pose3 prevPose_;

    // Functions
    void insCallback(const nav_msgs::Odometry &msg);
    void optimizeGraph();
    void publishPath(gtsam::Values &result);

public:
    OdomSlam(const ros::NodeHandle &nh);
    ~OdomSlam();
};

OdomSlam::OdomSlam(const ros::NodeHandle &nh) : nh_(nh)
{

    printf("Using ISAM2\n");
    gtsam::ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    isam2_ = new gtsam::ISAM2(parameters);
    lastInsTime_.fromSec(0);

    graph_.reset(new gtsam::NonlinearFactorGraph());
    initial_estimate_.reset(new gtsam::Values());

    // Give initial estimate for the extrinsics of the camera wrt robot

    // Initialize ROS subscribers and publishers
    subscriber_robot_pose_ =
        nh_.subscribe(ros_topic_odometry_, 2, &OdomSlam::insCallback, this);
    publisher_robot_optimized_path_ = nh_.advertise<nav_msgs::Path>(
        publisher_robot_optimized_path_topic_, 2, true);
    publisher_robot_poses_ = nh_.advertise<nav_msgs::Odometry>(
        publisher_robot_poses_topic_, 2, true);
}

OdomSlam::~OdomSlam() {}

// Store the last INS reading as a CPose3DPDFGaussian
// Note: We are assuming that INS readings happen at a very high frequency.
//       Therefore, interpolation between INS poses is not needed.
void OdomSlam::insCallback(const nav_msgs::Odometry &odomMsg)
{
    // newest_ins_pose_ = conversions::odometry_ros2mrpt(msg);
    rviz_ins_frame_ = odomMsg.header.frame_id;
    ros::Time curTime = ros::Time::now();
    int timeDiff = (curTime - lastInsTime_).toSec();
    if (timeDiff < insertPeriod_)
    {
        return;
    }

    robot_pose_counter_++;

    float p_x = odomMsg.pose.pose.position.x;
    float p_y = odomMsg.pose.pose.position.y;
    float p_z = odomMsg.pose.pose.position.z;
    float r_x = odomMsg.pose.pose.orientation.x;
    float r_y = odomMsg.pose.pose.orientation.y;
    float r_z = odomMsg.pose.pose.orientation.z;
    float r_w = odomMsg.pose.pose.orientation.w;
    gtsam::Pose3 currentPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

    if (robot_pose_counter_ == 0)
    {
        std::cout << "Inserting first odometry pose!" << std::endl;
        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m

        gtsam::PriorFactor<gtsam::Pose3> priorPose(X(robot_pose_counter_), currentPose, priorPoseNoise);
        graph_->add(priorPose);
        initial_estimate_->insert(X(robot_pose_counter_), currentPose);
    }
    else
    {
        // TODO: change initial guess;
        // insert between factor
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-1, 1e-1, 1e-1, 5e-1, 5e-1, 5e-1).finished());
        auto diff = prevPose_.between(currentPose);
        graph_->add(gtsam::BetweenFactor<gtsam::Pose3>(X(robot_pose_counter_ - 1), X(robot_pose_counter_), diff, odometryNoise));
        auto& prevPoseEstimate = result_.at(X(robot_pose_counter_-1)).cast<gtsam::Pose3>();
        initial_estimate_->insert(X(robot_pose_counter_), prevPoseEstimate.compose(diff));

        // Simulate GPS measurement and see the optimization result and uncertainties
        if (robot_pose_counter_ == 4)
        {
            std::cout << "Inserting first odometry pose!" << std::endl;
            gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
            priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(robot_pose_counter_), currentPose.compose(diff), priorPoseNoise);
            graph_->add(priorPose);
        }
    }

    prevPose_ = currentPose;
    lastInsTime_ = ros::Time::now();
    optimizeGraph();
}

void OdomSlam::optimizeGraph()
{

    if (debug_)
        graph_->print("Graph");

    // gtsam::Values result;

    std::cout << "Updating Graph!"  << std::endl;
    isam2_->update(*graph_, *initial_estimate_);
    isam2_->update();
    result_ = isam2_->calculateEstimate();

    std::cout << "Result: " << std::endl;
    result_.print();

    publishPath(result_);

    // reset the graph
    graph_->resize(0);
    initial_estimate_->clear();
}

void OdomSlam::publishPath(gtsam::Values &result)
{

    // Publish to visualize on RVIZ
    //   Robot poses
    nav_msgs::Path poses;
    for (size_t i = 0; i <= robot_pose_counter_; i++)
    {
        gtsam::Pose3 pose_robot_i =
            result.at(X(i)).cast<gtsam::Pose3>();
        Eigen::MatrixXd covariance = isam2_->marginalCovariance(X(i));

        geometry_msgs::PoseStamped p;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = rviz_ins_frame_;

        Eigen::Isometry3d eigen_pose;
        eigen_pose.setIdentity();
        eigen_pose.translate(Eigen::Vector3d(
            pose_robot_i.translation()[0], pose_robot_i.translation()[1],
            pose_robot_i.translation()[2]));
        eigen_pose.rotate(pose_robot_i.rotation().matrix());
        tf::poseEigenToMsg(eigen_pose, p.pose);
        poses.poses.push_back(p);

        boost::array<double, 36UL> cov_array;
        double* cov_carray = covariance.data();
        for (size_t i = 0; i < 36; i++)
        {
            cov_array[i] = cov_carray[i];
        }
        
        nav_msgs::Odometry odomI;
        odomI.pose.pose = p.pose;
        odomI.header = p.header;
        odomI.header.seq = i;
        odomI.pose.covariance = cov_array;
        publisher_robot_poses_.publish(odomI);
    }
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = rviz_ins_frame_;
    publisher_robot_optimized_path_.publish(poses);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_slam");
    ros::NodeHandle nh("~");

    OdomSlam node(nh);

    ros::spin();

    return 0;
}
