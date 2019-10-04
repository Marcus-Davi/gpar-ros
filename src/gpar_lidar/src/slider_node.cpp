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

const float Hz = 25; //Hz -> Frequencia do Sensor
const float Ts = 1/Hz;
float V_x; // m/s

void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
static float x = 0;	
static tf2_ros::TransformBroadcaster br; //tf Broadcaster
geometry_msgs::TransformStamped transformStamped;

x = x + V_x*Ts;

transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = "link";
transformStamped.child_frame_id = "map";


transformStamped.transform.translation.x = x; //programável
transformStamped.transform.translation.y = 0;
transformStamped.transform.translation.z = -3.0; //Altura pro chão

transformStamped.transform.rotation.w  = 1;
transformStamped.transform.rotation.x  = 0;
transformStamped.transform.rotation.y  = 0;
transformStamped.transform.rotation.z  = 0;


br.sendTransform(transformStamped); //Always Publish


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "slider");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  if (!nh_private.getParam("slider_x_vel", V_x))
  {
   V_x = 0.01;
   ROS_WARN("Set a velocidade do slider no .launch... setada para  \"%f\"",V_x);
  }

	
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud", 1, callback);


  ros::spin();
  return 0;
}




