#ifndef __RDS_PCD_TRANSFORM__
#define __RDS_PCD_TRANSFORM__

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

class TransformNode
{
  std::string target_frame_id;
  
  tf2_ros::Buffer * tf_buffer;
	tf2_ros::TransformListener * tf_listener;

  ros::Publisher pub_pcd;
  ros::Subscriber sub_pcd;

  private:


    Eigen::MatrixXd get_transform (std::string from, std::string to )
    {
      Eigen::MatrixXd ret(0,0);
      try
      {
        printf("looking for tranform %s --> %s\n",from.c_str(),to.c_str());
        geometry_msgs::TransformStamped transform = tf_buffer->lookupTransform( from, to, ros::Time::now(), ros::Duration(5.0) ); 
        printf("got transform\n");

		    Eigen::Vector3d t(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
            );

		    Eigen::Quaterniond q;
        q.x() = transform.transform.rotation.x;
        q.y() = transform.transform.rotation.y;
        q.z() = transform.transform.rotation.z;
        q.w() = transform.transform.rotation.w;
        q.normalize();
        Eigen::Matrix3d R(q);

        Eigen::Matrix4d T;
        T.block(0,0,3,3) = R;
        T.block(0,3,3,1) = t;
        T.block(3,0,1,4) << 0, 0, 0, 1;

        ret = T.inverse();

        std::cout << T << std::endl;
      }
      catch(tf2::TransformException ex)
      {
        ros::Duration(0.1).sleep();
        printf("TF ERROR: transform from %s to %s\n", from.c_str(), to.c_str() );
      }
      return ret;
    }

    void publish_pcd(Eigen::MatrixXd& pcd, double ts, std::string frame_id, const sensor_msgs::PointCloud2::ConstPtr& msg, std::vector<uint32_t> times)
    {
      sensor_msgs::PointCloud2 pc2msg;

      //generate x field for rviz
      // sensor_msgs::PointField pf_x;
      // pf_x.name = "x";
      // pf_x.datatype = 7;
      // pf_x.offset = 0;
      // pf_x.count = 1;
      // //generate y field for rviz
      // sensor_msgs::PointField pf_y;
      // pf_y.name = "y";
      // pf_y.datatype = 7;
      // pf_y.offset = 4;
      // pf_y.count = 1;
      // //generate z field for rviz
      // sensor_msgs::PointField pf_z;
      // pf_z.name = "z";
      // pf_z.datatype = 7;
      // pf_z.offset = 8;
      // pf_z.count = 1;
      // //generate t field for rviz
      // sensor_msgs::PointField pf_t;
      // pf_t.name = "t";
      // pf_t.datatype = 6;
      // pf_t.offset = 12;
      // pf_t.count = 1;

      // pc2msg.fields.push_back(pf_x); 
      // pc2msg.fields.push_back(pf_y); 
      // pc2msg.fields.push_back(pf_z);
      // pc2msg.fields.push_back(pf_t);  
      pc2msg.fields = msg->fields;

      for (int i = 0; i < pcd.cols(); ++i)
      {
        float x_val = pcd(0,i);
        for (uint8_t * byteptr = (uint8_t *) &x_val; byteptr < ((uint8_t *) &x_val) + 4; byteptr++) pc2msg.data.push_back(*byteptr);
        float y_val = pcd(1,i);
        for (uint8_t * byteptr = (uint8_t *) &y_val; byteptr < ((uint8_t *) &y_val) + 4; byteptr++) pc2msg.data.push_back(*byteptr);
        float z_val = pcd(2,i);
        for (uint8_t * byteptr = (uint8_t *) &z_val; byteptr < ((uint8_t *) &z_val) + 4; byteptr++) pc2msg.data.push_back(*byteptr);
        uint32_t t_val = times[i];
        for (uint8_t * byteptr = (uint8_t *) &t_val; byteptr < ((uint8_t *) &t_val) + 4; byteptr++) pc2msg.data.push_back(*byteptr);
      }

      pc2msg.point_step = msg->point_step;
      pc2msg.row_step = msg->row_step;
      pc2msg.height = msg->height;
      pc2msg.width = msg->width;

      pc2msg.header.frame_id = frame_id;
      // pc2msg.header.stamp = ros::Time(ts);
      pc2msg.header.stamp = (msg->header.stamp);
      pc2msg.is_dense = msg->is_dense;

      pub_pcd.publish(pc2msg);
    }

    Eigen::MatrixXd parse_coords(const sensor_msgs::PointCloud2::ConstPtr& msg, std::vector<uint32_t>& times)
    {
      sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<uint32_t> it_t(*msg, "t");

      int n = msg->width*msg->height;
      
      Eigen::MatrixXd coords(4,n);

      for (int i = 0; i < n; ++i)
      {
        coords(0,i) = *it_x;
        coords(1,i) = *it_y;
        coords(2,i) = *it_z;
        coords(3,i) = 1;
        times.push_back(*it_t);

        ++it_x;++it_y;++it_z;
        ++it_t;
      }

      return coords;
    }

    void callback_pcd(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      std::string last_frame_id = msg->header.frame_id;
      double ts = msg->header.stamp.toSec();
      
      
      Eigen::MatrixXd transform = get_transform(last_frame_id, target_frame_id);

      if (transform.size())
      {
        std::vector<uint32_t> times;
        Eigen::MatrixXd coords = parse_coords(msg, times);
        int n = coords.cols();
        std::cout << coords.block(0,n/2-10,4,10) << std::endl;
        coords = transform * coords;
        std::cout << coords.block(0,n/2-10,4,10) << std::endl;

        publish_pcd(coords, ts, target_frame_id, msg, times);
      }

    }

  public:


    TransformNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {


      tf_buffer = new tf2_ros::Buffer();
	    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

      pnh.getParam("target_frame",target_frame_id);
      printf("target_frame: %s\n",target_frame_id.c_str());

      pub_pcd = nh.advertise<sensor_msgs::PointCloud2>("out", 1);
      sub_pcd = nh.subscribe( "in", 1, &TransformNode::callback_pcd, this);
    }
    

};

int init_pcd_transform_node(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_transform_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  TransformNode tp_node(nh,pnh);
  
  ros::spin();
  return 0;
}

#endif 
