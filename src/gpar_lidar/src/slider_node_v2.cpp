/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>   // necessary because of custom point type

// Conversão mm/min -> m/s

// x (m/s) = y(mm/min) / (60000)


void callback_serial(const std_msgs::String::ConstPtr& msg){
float x;
float y;
char* start;
//sscanf(msg->data.c_str(),"WPos:%f",&y);
if(( start = strstr((char*)msg->data.c_str(),"WPos:") ) != NULL){
sscanf(start,"WPos:%f,%f",&x,&y);
} else {

}

//ROS_INFO("msg : %s",msg->data.c_str());

//ROS_INFO("sub msg : %s",start);

//ROS_INFO("x : %f y : %f",x,y);


static tf2_ros::TransformBroadcaster br; //tf Broadcaster
geometry_msgs::TransformStamped transformStamped;


transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = "link";
transformStamped.child_frame_id = "map";


transformStamped.transform.translation.x = x/1000; //milimetros -> metros
transformStamped.transform.translation.y = y/1000;
transformStamped.transform.translation.z = -3.0; //Altura pro chão

transformStamped.transform.rotation.w  = 1;
transformStamped.transform.rotation.x  = 0;
transformStamped.transform.rotation.y  = 0;
transformStamped.transform.rotation.z  = 0;


br.sendTransform(transformStamped); //Always Publish
}



void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{



}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "slider");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

	
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud", 1, callback);
  ros::Subscriber sub_string = nh.subscribe ("mcuserial_node_gcode/serial_data", 1, callback_serial);


  ros::spin();
  return 0;
}




