#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <eigen_conversions/eigen_msg.h>

bool init = true;
ros::Publisher odom_pub;
Eigen::Affine3d firstTransform;


std::vector<double> odometerTransV;
std::vector<double> odometerRotV;
Eigen::Vector3d odometerTrans;
Eigen::Quaterniond odometerRot;
Eigen::Affine3d odometerTf(Eigen::Affine3f::Identity());

std::string odomFrame = "odom";
std::string baseFrame = "base_link";

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    nav_msgs::Odometry msgOut(*msg);
    
    Eigen::Affine3d currentTransform;
    tf::poseMsgToEigen(msg->pose.pose, currentTransform);

    // std::cout << "Input transform: " << std::endl << currentTransform.matrix();
    currentTransform = currentTransform * odometerTf;
    //  std::cout << "Input transform corrected: " << std::endl << currentTransform.matrix();

    if (init)
    {
        firstTransform = currentTransform;
        std::cout << "First transform: " << std::endl << firstTransform.matrix();
        init = false;
    }
    
    currentTransform = firstTransform.inverse() * currentTransform;

    // std::cout << "Current transform: " << std::endl << currentTransform.matrix();

    tf::poseEigenToMsg (currentTransform, msgOut.pose.pose);
    msgOut.child_frame_id=baseFrame;
    msgOut.header.frame_id=odomFrame;
    odom_pub.publish(msgOut);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_gt_publisher");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    std::string topicIn = "odom";
    std::cout << "Input topic name: " << topicIn << std::endl;
    std::string topicOut = "odom_out";
    std::cout << "Output topic name: " << topicOut << std::endl;
    

    pnh.getParam("odomFrame", odomFrame);
    pnh.getParam("baseFrame", baseFrame);

    odometerTransV = {0, 0, 0};
    odometerRotV = {0, 0, 0, 1};
    pnh.param<std::vector<double>>("odometerTrans", odometerTransV, odometerTransV);
    pnh.param<std::vector<double>>("odometerRot", odometerRotV, odometerRotV);
    odometerTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(odometerTransV.data(), 3, 1);
    odometerRot = Eigen::Quaterniond(odometerRotV.data());

    odometerTf.rotate(odometerRot);
    odometerTf.translate(odometerTrans);

    std::cout << "odometer transform: " << std::endl << odometerTf.matrix() << std::endl;

    odom_pub = n.advertise<nav_msgs::Odometry>(topicOut, 10);
    ros::Subscriber odom_sub = n.subscribe(topicIn, 10, odomCallback);

    std::cout << "Using Frames:" << std::endl;
    std::cout << "Odom Frame: " << odomFrame << std::endl;
    std::cout << "Base Frame: " << baseFrame << std::endl;

    ros::spin();
    return 0;
}