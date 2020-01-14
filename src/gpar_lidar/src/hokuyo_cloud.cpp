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

laser_geometry::LaserProjection projector;
ros::Publisher pub;
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){

sensor_msgs::PointCloud2 cloud;
projector.projectLaser(*scan_in,cloud);


cloud.header.frame_id = "cloud";
cloud.header.stamp = ros::Time::now();
pub.publish(cloud);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_controller");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  laser_geometry::LaserProjection projector_;

  std::string output_cloud_name;
  std::string scan_topic_name;

  if(!private_nh.getParam("scan_topic",scan_topic_name))
   {
     scan_topic_name = "scan";
     ROS_WARN("Need to set parameter 'scan_topic_name'.. set to \"%s\"",scan_topic_name.c_str());
   }
  if(!private_nh.getParam("input_cloud",output_cloud_name))
   {
     output_cloud_name = "cloud";
     ROS_WARN("Need to set parameter 'output_cloud_name'.. set to \"%s\"",output_cloud_name.c_str());
   }

   ros::Subscriber sub = nh.subscribe(scan_topic_name,10,laser_callback);
    pub  = nh.advertise<sensor_msgs::PointCloud2>(output_cloud_name, 10);


  ros::spin();



  return 0;
}
