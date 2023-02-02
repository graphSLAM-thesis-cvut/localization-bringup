#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <eigen_conversions/eigen_msg.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>

#include <visualization_msgs/MarkerArray.h>

using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

// using namespace graph_slam_3d;

class OdomSlam
{
private:
    // ROS variables
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_robot_pose_;
    ros::Publisher publisher_robot_optimized_path_;
    ros::Publisher publisher_landmarks_;
    ros::Time lastInsTime_;

    // ROS topics
    std::string ros_topic_odometry_ = "/camera/odom/sample";
    std::string publisher_robot_optimized_path_topic_ = "path_optimized";

    // GTSAM variables

    // ExpressionFactorGraph is a NonLinearFactorGraph that can
    // *additionally* use expressions as factors. This is useful
    // for us because it allows us to work with BearingRange
    // expressions
    gtsam::NonlinearFactorGraph::shared_ptr graph_;

    gtsam::Values::shared_ptr initial_estimate_;
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
        // insert prior factor

        // conversions::pose_mrpt2gtsam(newest_ins_pose_, pose, cov);
        // mrpt::gtsam_wrappers::to_gtsam_se3_cov6(newest_ins_pose_, pose, cov);

        // graph_->addExpressionFactor(
        // 	gtsam::noiseModel::Gaussian::Covariance(cov), pose,
        // 	gtsam::Pose3_('x', robot_pose_counter_));
        // initial_estimate_->insert(
        // 	gtsam::Symbol('x', robot_pose_counter_), pose);
        // last_ins_pose_ = newest_ins_pose_;
    }
    else
    {
        // TODO: change initial guess;
        std::cout << "Inserting " << robot_pose_counter_ << " odometry pose and between factor!" << std::endl;
        // insert between factor
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        graph_->add(gtsam::BetweenFactor<gtsam::Pose3>(X(robot_pose_counter_ - 1), X(robot_pose_counter_), prevPose_.between(currentPose), odometryNoise));
        std::cout << "Inserting initial estimate for pose " << X(robot_pose_counter_) << std::endl;
        initial_estimate_->insert(X(robot_pose_counter_), currentPose);
    }

    prevPose_ = currentPose;
    lastInsTime_ = ros::Time::now();
    optimizeGraph();
}

// This is the main callback of the node.
// When we receive a message with the transformation of an aruco wrt the camera,
// we add to the graph:
//    - A new node with the current robot pose
//    - If the aruco had not been previously seen, a node with the aruco pose
//    - A "between" factor (odometry) between the current and last robot poses
//    (computed with MRPT)
//    - A "bearingRange" expression factor between the current robot pose node
//    and the observed landmark node
// Finally, the graph is optimized after every observation and the results
// published
void OdomSlam::optimizeGraph()
{

    // The position of the fiducial relative to the camera.
    // We are currently not using the orientation of the aruco but
    // should be easy to implement if needed (betweenfactor).

    // Create odometry node and factor

    if (debug_)
        graph_->print("Graph");

    gtsam::Values result;


    std::cout << "Updating Graph!"  << std::endl;
    isam2_->update(*graph_, *initial_estimate_);
    isam2_->update();
    result = isam2_->calculateEstimate();

    std::cout << "Result: " << std::endl;
    result.print();
    // Calculate and print marginal covariances for all variables
    // gtsam::Marginals marginals(*graph_, result);

    // Publish to visualize on RVIZ
    //   Robot poses
    nav_msgs::Path poses;
    for (size_t i = 0; i <= robot_pose_counter_; i++)
    {

        std::cout << "Getting pose " << X(i) << std::endl;
        gtsam::Pose3 pose_robot_i =
            result.at(X(i)).cast<gtsam::Pose3>();
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
    }
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = rviz_ins_frame_;
    publisher_robot_optimized_path_.publish(poses);


    // reset the graph
    graph_->resize(0);
    initial_estimate_->clear();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_slam");
    ros::NodeHandle nh("~");

    OdomSlam node(nh);

    ros::spin();

    return 0;
}
