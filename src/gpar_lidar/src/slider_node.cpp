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
#include <message_filters/subscriber.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>   // necessary because of custom point type

tf::TransformListener* tf_;
std::string fixed_frame_;


void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{



}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "slider");
  ros::NodeHandle nh;


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("cloud", 1, callback);


  ros::spin();
  return 0;
}




