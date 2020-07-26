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
#include <pcl/filters/passthrough.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <std_msgs/UInt8.h>

tf2_ros::TransformListener *tf_listener;
tf2_ros::Buffer *tf_buffer;



typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

bool start_merge = false;
float x_min;
float x_max;

// Nuvem Global
PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();
ros::Publisher merge_signal;


// Callback
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr pc){


boost::shared_ptr<PointCloudT> cloud = boost::make_shared<PointCloudT>();
boost::shared_ptr<PointCloudT> cloud_filtered = boost::make_shared<PointCloudT>();

pcl::fromROSMsg(*pc,*cloud);





/* Ideia
* filtrar pontos acima do chão (hardcode < 1.4 m)
* tirar media
* gerar posição no chão com a media

*/
//Processar nuvem
	pcl::PassThrough<pcl::PointXYZ> pfilter;
  pcl::PointXYZ min,max;
	pfilter.setInputCloud(cloud);
	pfilter.setFilterFieldName("x");
	pfilter.setFilterLimits(0,1.4); //hardcoded floor
	pfilter.filter(*cloud_filtered);

  int n = cloud_filtered->size();
  float x_avg = 0;
  float y_avg = 0;
  for(int i=0;i<n;++i){
    y_avg += cloud_filtered->points[i].y;

  }

  y_avg /= n;
  
  geometry_msgs::PointStamped position_in;
  position_in.point.x = 1.5; //hardcoded floor
  position_in.point.y = y_avg;
  position_in.point.z = 0;

  geometry_msgs::PointStamped position_out;
  
  geometry_msgs::TransformStamped tf_point = tf_buffer->lookupTransform("map",pc->header.frame_id,ros::Time(0));
  tf2::doTransform(position_in,position_out,tf_point);


  //ROS_INFO("in %f %f %f",position_in.point.x,position_in.point.y,position_in.point.z);
  //ROS_INFO("ou %f %f %f",position_out.point.x,position_out.point.y,position_out.point.z);




float x = y_avg;
ROS_INFO("x = %f",x);

std_msgs::UInt8 msg;
if(x < x_max && x > x_min){ // AQUI DEFINIMOS A FAIXA. PODE SER UM MIX COM SENSOR TOP TBM! PARAMETRIZAR 
  start_merge = true;
  msg.data = 1;
} else {
  start_merge = false;
    msg.data = 0;
}

merge_signal.publish(msg);

// Geramos um frame na posição desse x
static tf2_ros::TransformBroadcaster br;
geometry_msgs::TransformStamped transformStamped;

transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = "map";
transformStamped.child_frame_id = "obj";

tf2::Quaternion q;
q.setRPY(0,0,0);
geometry_msgs::Quaternion q_msg = tf2::toMsg(q);

transformStamped.transform.rotation = q_msg;

transformStamped.transform.translation.x = position_out.point.x;
transformStamped.transform.translation.y = position_out.point.y;
transformStamped.transform.translation.z = 0;

br.sendTransform(transformStamped);
//ROS_INFO("Transform");


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bulk_scan_profiler");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber front_laser = nh.subscribe("/cloud_profiler",10,lidar_callback);
  merge_signal = nh.advertise<std_msgs::UInt8>("merge_signal",5);
  
  


  if (! private_nh.getParam("x_min",x_min) ) {
    ROS_ERROR("sete o parametro 'x_min'!");
    return 1;
  }

  if (! private_nh.getParam("x_max",x_max) ) {
    ROS_ERROR("sete o parametro 'x_max'!");
    return 1;

  }
  
  tf_buffer = new tf2_ros::Buffer;
	tf_listener = new tf2_ros::TransformListener(*tf_buffer);

  

  ros::spin();


  delete tf_buffer;
  delete tf_listener;
}


