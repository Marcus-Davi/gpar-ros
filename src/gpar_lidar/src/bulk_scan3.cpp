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

#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <std_msgs/UInt8.h>

ros::Publisher merged_cloud;

tf2_ros::TransformListener *tf_listener;
tf2_ros::Buffer *tf_buffer;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
float vel;
float x0 = 0;
ros::Time t0;
bool first = false;

bool start_merge = false;

// Interval
float x_min;
float x_max;


// Nuvem Global
PointCloudT::Ptr cloud_object = boost::make_shared<PointCloudT>();

void signal_callback(const std_msgs::UInt8ConstPtr msg){
  if(msg->data == 1){
    start_merge = true;
  } else  {
  start_merge = false;
}


}


void top_callback(const sensor_msgs::PointCloud2ConstPtr pc1,const sensor_msgs::PointCloud2ConstPtr pc2){
if(start_merge){
  ROS_WARN("merging..");
} else {
  ROS_WARN("waiting..");
  cloud_object->clear();
}

ROS_INFO("synced");

if(!start_merge)
return;


PointCloudT::Ptr cloud_top1 = boost::make_shared<PointCloudT>();
PointCloudT::Ptr cloud_top2 = boost::make_shared<PointCloudT>();
PointCloudT::Ptr cloud_map = boost::make_shared<PointCloudT>();

PointCloudT::Ptr cloud_map_output = boost::make_shared<PointCloudT>();



pcl::fromROSMsg(*pc1,*cloud_top1);
pcl::fromROSMsg(*pc2,*cloud_top2);



// Transforma

try { 
//geometry_msgs::TransformStamped tf = tf_buffer->lookupTransform("map",pc->header.frame_id,ros::Time(0));

pcl_ros::transformPointCloud("map",*cloud_top1,*cloud_top1,*tf_buffer);
pcl_ros::transformPointCloud("map",*cloud_top2,*cloud_top2,*tf_buffer);


*cloud_map = *cloud_top1 + *cloud_top2; // junta as 2

// O "map" tem origem imediatamente abaixo do cloud_top
geometry_msgs::TransformStamped tf_map_obj = tf_buffer->lookupTransform("map","obj",ros::Time(0));


/* A Ideia :
1) Transforma nuvem "top" segundo frame "map". chamo de "cloud_map"
2) Um quadro móvel, "obj", se desloca junto com objeto a ser escaneado.
3) Translada nuvem "cloud_obj" junto com objeto. A "cloud_object" contem todo histórico de medidas
4) Grava/agrega/soma/junta a nuvem "cloud_map" em "cloud_object". chamo de "cloud_object"
3) 

*/


//Translada
pcl_ros::transformPointCloud(*cloud_map,*cloud_map,tf_map_obj.transform);

// Funde. Por algum motivo a nuvem tá sendo duplicada no map e no obj
*cloud_object += *cloud_map;


} catch(tf2::TransformException &ex) {
ROS_WARN("%s", ex.what());
}

// Apenas espelhar
geometry_msgs::Transform mirror_tf ;
tf2::Quaternion q_;
q_.setRPY(0,0,M_PI);
mirror_tf.rotation = tf2::toMsg(q_);
mirror_tf.translation.x = 0;
mirror_tf.translation.y = 0;
mirror_tf.translation.z = 0;

//pcl_ros::transformPointCloud(*cloud_object,*cloud_object,mirror_tf);


sensor_msgs::PointCloud2Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();

pcl::toROSMsg(*cloud_object,*cloud_msg);



cloud_msg->header.frame_id = "map";
cloud_msg->header.stamp = ros::Time::now();


merged_cloud.publish(cloud_msg);

// Merge 
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "bulk_scan");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber merge_signal = nh.subscribe("merge_signal",5,signal_callback);



  //Usar message filter
  	message_filters::Subscriber<sensor_msgs::PointCloud2> pc1_sub(nh,"/cloud_top1",1);
		message_filters::Subscriber<sensor_msgs::PointCloud2> pc2_sub(nh,"/cloud_top2",1);
  
  	//message_filters::TimeSynchronizer<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> sync(pc1_sub,pc2_sub,10); //pq n funciona ?


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,sensor_msgs::PointCloud2> MySyncPolicy;	
  message_filters::Synchronizer<MySyncPolicy> sync2(MySyncPolicy(10),pc1_sub,pc2_sub);

		sync2.registerCallback(boost::bind(&top_callback,_1,_2));


		

  merged_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_merged",10);

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

  return 0;
}


