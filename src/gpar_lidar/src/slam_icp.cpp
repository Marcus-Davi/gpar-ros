/* Author : Marcus Forte <davi2812@dee.ufc.br> */

#include <ros/ros.h> //General Ros stuff
#include <sensor_msgs/PointCloud2.h> // Process PC2 message
#include <pcl/point_types.h> // PCL types .. i.e PointXYZ
#include <pcl_conversions/pcl_conversions.h> //PCL to ROS | ROS to PCL
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h" // Allows direct transform application on ros message

#include "std_msgs/String.h" //receive messages to manipulate cloud 

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT; // Define a templated type of pointcloud

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
ROS_INFO("got cloud!");

}


int main(int argc,char** argv){
	ros::init(argc,argv,"slam_icp");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	
	std::string cloud_topic = nh.resolveName("cloud");

	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic,1,cloud_callback);


	ros::spin();


}
