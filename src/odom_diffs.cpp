#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <iostream>
#include <fstream>

std::ofstream myfile;
double startTime = 0;

void odomCallback(const nav_msgs::OdometryConstPtr& odom1, const nav_msgs::OdometryConstPtr& odom2)
{
  
  auto& p1 = odom1->pose.pose.position; 
  auto& p2 = odom2->pose.pose.position;
  
  myfile.open("diffs.txt", std::ifstream::app);
  double time = odom1->header.stamp.toSec();
  if (startTime == 0){
    startTime = time;
  }
  time -= startTime;
  double diff = std::sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z)) ; 
  std::cout << time << ": "<< diff << std::endl;
  myfile << time << " " << diff << std::endl;
  myfile.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_comparator");
    ros::NodeHandle nh;
    myfile.open("diffs.txt", std::ifstream::out | std::ifstream::trunc);
    myfile.close();

    message_filters::Subscriber<nav_msgs::Odometry> odom1_sub(nh, "/odom1", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom2_sub(nh, "/odom2", 1);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom1_sub, odom2_sub);
    sync.registerCallback(boost::bind(&odomCallback, _1, _2));


    ros::spin();
    return 0;
}