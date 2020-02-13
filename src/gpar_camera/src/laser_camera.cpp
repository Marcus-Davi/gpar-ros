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
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>

typedef PointCloudT pcl::PointXYZRGB;

sensor_msgs::Image image_msg;
sensor_msgs::PointCloud2 pc_msg;


ros::Subscriber sub_pc;
ros::Subscriber sub_img;
ros::Publisher pub_pc;

sensor_msgs::PointCloud2::Ptr cloud = boost::make_shared<sensor_msgs::PointCloud2>();

void img_callback(const sensor_msgs::Image::ConstPtr& img_msg){
ROS_INFO("got IMG");
PointCloudT::Ptr cloud_color = boost::make_shared<PointCloudT>();
//Sincronizar msgs


}

void pc_callback(const sensor_msgs::PointCloud2::Ptr pc_msg){
ROS_INFO("got Cloud");
cloud = pc_msg;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_camera");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string input_cloud_name;
  std::string input_image;


  if(!private_nh.getParam("input_cloud",input_cloud_name))
   {
     input_cloud_name = "cloud";
     ROS_WARN("Need to set parameter 'output_cloud_name'.. set to \"%s\"",input_cloud_name.c_str());
   }

   if(!private_nh.getParam("input_image",input_image))
    {
      input_image = "image";
      ROS_FATAL("falout node da image RUIM!");
      return -1;
    }

    sub_pc = nh.subscribe(input_cloud_name,10,pc_callback);
    sub_img = nh.subscribe(input_image,10,img_callback);

    pub_pc = nh.advertise<sensor_msgs::PointCloud2>("color_cloud",10);


  ros::spin();



  return 0;
}
