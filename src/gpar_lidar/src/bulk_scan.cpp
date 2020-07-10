/*
 * Copyright (C) 2016, DFKI GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of DFKI GmbH nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Authors:
 *         Martin Günther <martin.guenther@dfki.de>
 *         Jochen Sprickerhof <ros@jochen.sprickerhof.de>
 *
 */

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <message_filters/subscriber.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>   // necessary because of custom point type


#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

ros::Publisher merged_cloud;

tf2_ros::TransformListener *tf_listener;
tf2_ros::Buffer *tf_buffer;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
float vel;
float x0 = 0;
ros::Time t0;
bool first = false;

bool start_merge = false;


// Nuvem Global
PointCloudT::Ptr cloud_merged = boost::make_shared<PointCloudT>();



void top_callback(const sensor_msgs::PointCloud2ConstPtr pc){
if(start_merge){
  ROS_WARN("merging..");
} else {
  ROS_WARN("waiting..");
  cloud_merged->clear();
}

if(!start_merge)
return;


PointCloudT::Ptr cloud_in = boost::make_shared<PointCloudT>();
PointCloudT::Ptr cloud_map = boost::make_shared<PointCloudT>();

PointCloudT::Ptr cloud_map_output = boost::make_shared<PointCloudT>();



pcl::fromROSMsg(*pc,*cloud_in);


// Transforma

try { 
//geometry_msgs::TransformStamped tf = tf_buffer->lookupTransform("map",pc->header.frame_id,ros::Time(0));

pcl_ros::transformPointCloud("map",*cloud_in,*cloud_map,*tf_buffer);
// agora desloca junto com frame obj
geometry_msgs::TransformStamped tf_map_obj = tf_buffer->lookupTransform("map","obj",ros::Time(0));

// TODO PENSAR !!!!!!!
*cloud_merged += *cloud_map; 

pcl_ros::transformPointCloud(*cloud_merged,*cloud_merged,tf_map_obj.transform);


} catch(tf2::TransformException &ex) {
ROS_WARN("%s", ex.what());
}





sensor_msgs::PointCloud2Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
pcl::toROSMsg(*cloud_merged,*cloud_msg);
cloud_msg->header.frame_id = "map";
cloud_msg->header.stamp = ros::Time::now();
merged_cloud.publish(cloud_msg);
// Merge 


}

void front_callback(const sensor_msgs::PointCloud2ConstPtr pc){
// Calcular velocidade
if(first == false){
  t0 = ros::Time::now();
  first = true;
  return;
}

ros::Time t = ros::Time::now();
ros::Duration dt = t - t0;
t0 = t;


boost::shared_ptr<PointCloudT> cloud = boost::make_shared<PointCloudT>();

pcl::fromROSMsg(*pc,*cloud);

int size_2 = cloud->size()/2;
if(size_2 == 0)
return;


float x = cloud->points[size_2].x;
ROS_INFO("x = %f",x);

// A partir de x=1.6, grava nuvem
if(x < 1.7 && x > 0.7){ // AQUI DEFINIMOS A FAIXA. PODE SER UM MIX COM SENSOR TOP TBM! PARAMETRIZAR 
  start_merge = true;
} else {
  start_merge = false;
}

// Geramos um frame na posição desse x
static tf2_ros::TransformBroadcaster br;
geometry_msgs::TransformStamped transformStamped;

transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = "cloud_front";
transformStamped.child_frame_id = "obj";

tf2::Quaternion q;
q.setRPY(0,0,M_PI);
geometry_msgs::Quaternion q_msg = tf2::toMsg(q);

transformStamped.transform.rotation = q_msg;

transformStamped.transform.translation.x = x;
transformStamped.transform.translation.y = 0;
transformStamped.transform.translation.z = 0;

br.sendTransform(transformStamped);
ROS_INFO("Transform");


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bulk_scan");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber front_laser = nh.subscribe("/cloud_front",10,front_callback);
  ros::Subscriber top_laser = nh.subscribe("/cloud_top",10,top_callback);

  merged_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_merged",10);


  	tf_buffer = new tf2_ros::Buffer;
	tf_listener = new tf2_ros::TransformListener(*tf_buffer);

  

  ros::spin();
}


