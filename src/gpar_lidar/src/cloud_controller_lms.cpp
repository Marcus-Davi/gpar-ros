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

#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>   // necessary because of custom point type

#include <gpar_lidar/Command.h>

#include <fstream> //file save
using namespace std;
ofstream myfile;





typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

ros::Publisher pub_;

tf::TransformListener* tf_;
std::string fixed_frame_;
PointCloudT::Ptr cloud_lower_ = boost::make_shared<PointCloudT>();

void savePoints(PointCloudT::Ptr pc){
 ros::Time time = ros::Time::now();

char* home_path = getenv("HOME");
std::string str_path = std::string(home_path) + "/pontos_";
std::string str_time = std::to_string(time.sec);
std::string str = str_path + str_time + ".txt";

myfile.open(str); 
//ROS_INFO("file n = %s",str.c_str());

for (size_t i = 0; i < pc->points.size(); i++) {
       myfile << pc->points[i].x << " " << pc->points[i].y << " " <<pc->points[i].z << endl;
//ROS_INFO("%f",pc->points[i].x);
  }

myfile.close();
ROS_INFO("%lu Pontos salvos em %s !\n",pc->points.size(),str.c_str());


}

bool StartAggregation = false;



void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
if(!StartAggregation)
return;


  PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();
  pcl::fromROSMsg(*pc, *cloud);
  // ----- transform to fixed frame
  try
  {
    PointCloudT::Ptr cloud_fixed = boost::make_shared<PointCloudT>();

    if (!pcl_ros::transformPointCloud(fixed_frame_, *cloud, *cloud_fixed, *tf_))
    {
      ROS_WARN("TF exception in transformPointCloud!");
      return ;
    }
    cloud = cloud_fixed;
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("TF Exception %s", ex.what());
    return ;
  }

  // ----- concatenate lower + upper clouds
  *cloud_lower_ += *cloud;


  // ----- publish
  sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_lower_, *msg);
  msg->header.stamp = pc->header.stamp;
  msg->header.frame_id = fixed_frame_;
  pub_.publish(msg);
}

//ler inteiro
bool command_parser(gpar_lidar::Command::Request &req,
	  	    gpar_lidar::Command::Response &res)
 {
   res.command_res = 0;

  switch(req.command){
   case 0:
   ROS_INFO("Stop Merging.. \n");
   StartAggregation = false;
   res.command_res = 1;
   break;
   case 1:
   ROS_INFO("Merging clouds.. \n");
   StartAggregation = true;
   res.command_res = 1;
   break;
   case 2:
   ROS_INFO("Restarting cloud.. \n");
   cloud_lower_->clear();
   res.command_res = 1;
   break;
   case 3:
   ROS_INFO("Saving cloud.. \n");
   savePoints(cloud_lower_);
   res.command_res = 1;
   break;
   default:
   break;

   }


  return true;
 }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_controller");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string input_cloud_node;

  if (!private_nh.getParam("fixed_frame", fixed_frame_))
  {
   fixed_frame_ = "map";
   ROS_WARN("Need to set parameter fixed_frame.. set to \"%s\"",fixed_frame_.c_str());
  }

  if(!private_nh.getParam("input_cloud",input_cloud_node))
   {
    ROS_FATAL("Need to set input node to subscribe to!"); 
    return 1;
   }

// SERIVCE

 ros::ServiceServer service = private_nh.advertiseService("command_parser", command_parser);

  tf_ = new tf::TransformListener(nh, ros::Duration(3.0));

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub;
  tf::MessageFilter<sensor_msgs::PointCloud2>* tf_filter;

  sub.subscribe(nh, input_cloud_node, 10);
  tf_filter = new tf::MessageFilter<sensor_msgs::PointCloud2>(sub, *tf_, fixed_frame_, 10);
  tf_filter->registerCallback(boost::bind(callback, _1));

  pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 10);

  ros::spin();

  delete tf_filter;
  delete tf_;

  return 0;
}
