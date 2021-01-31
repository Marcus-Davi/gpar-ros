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

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

//sensor_msgs::Image image_msg;
sensor_msgs::PointCloud2 pc_msg;


ros::Subscriber sub_pc;
ros::Subscriber sub_img;
ros::Publisher pub_pc;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

void img_callback(const sensor_msgs::Image::ConstPtr& img_msg){
ROS_INFO("got IMG");
//Sincronizar msgs
PointCloudT::Ptr cloud_color = pcl::make_shared<PointCloudT>();
size_t pc_size = cloud->size();
cloud_color->resize(pc_size);

//Aqui preciso acessar a linha Y da imagem
// Faz Y = 240
// Acesso da ROW -> 1920*row. Row(240) = 1920*240.
// Tamanho da Row = 1920 bytes.
// Conversão pixel-> ponto
// HokuyoRES -> 0.36°
// CameraRES -> 0.0625
// Conversão -> 5.76! (A cada ponto do lidar, incrementamos ~6*3 pontos camera)

// [RGB] [RGB] ... [RGB]
// .                 .
// .                 .    {480
// .                 .
// [RGB] [RGB] .. [RGB]
//      640
// Como mapear 640 pontos para pc_size ?


for (unsigned int i = 0;i<pc_size;++i){
  cloud_color->points[i].x = cloud->points[i].x;
  cloud_color->points[i].y = cloud->points[i].y;
  cloud_color->points[i].z = cloud->points[i].z;
  //aqui preciso acessar a imagem na linha correta. por algum motivo, os pontos tão invertidos com as cores
  cloud_color->points[pc_size-i-1].r = img_msg->data[1920*240 + 18*i];
  cloud_color->points[pc_size-i-1].g = img_msg->data[1920*240 + 18*i+1];
  cloud_color->points[pc_size-i-1].b = img_msg->data[1920*240 + 18*i+2];
  // cloud_color->points[i].a = 0.7;
  //ROS_INFO("size = %d",1920*240 + 6*i);
}

sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
pcl::toROSMsg(*cloud_color, *msg);
msg->header.stamp = ros::Time::now();
msg->header.frame_id = cloud->header.frame_id;
pub_pc.publish(msg);



}

void pc_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
ROS_INFO("got Cloud");
pcl::fromROSMsg(*pc_msg, *cloud);
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
     ROS_WARN("Need to set parameter 'input_cloud'.. set to \"%s\"",input_cloud_name.c_str());
   }

   if(!private_nh.getParam("input_image",input_image))
    {
      ROS_FATAL("param _input_image n definido!");
      ROS_WARN("saindo .. ");
      return -1;
    }

    sub_pc = nh.subscribe(input_cloud_name,10,pc_callback);
    sub_img = nh.subscribe(input_image,10,img_callback);

    pub_pc = nh.advertise<sensor_msgs::PointCloud2>("color_cloud",10);


  ros::spin();



  return 0;
}
