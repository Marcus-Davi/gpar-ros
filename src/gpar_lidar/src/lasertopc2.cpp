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
std::string cloud_frame_name;

double angle = 0;


void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
static sensor_msgs::PointCloud2 cloud;
static sensor_msgs::LaserScan scan_out;
double angle_abs = fabs(angle);

//Filtrar angulo
if(angle != 0){
  double start_angle = scan_in->angle_min;
  double current_angle = scan_in->angle_min;
  scan_out.angle_increment = scan_in->angle_increment;
  scan_out.ranges.resize(scan_in->ranges.size());
  ros::Time start_time = scan_in->header.stamp;
unsigned int count = 0;
//Varre toda a entrada
  for(unsigned int i=0;i < scan_in->ranges.size(); ++i){
    if(start_angle < -angle_abs){
      start_angle += scan_in->angle_increment;
      current_angle += scan_in->angle_increment;
      start_time += ros::Duration(scan_in->time_increment);
    } else {
      scan_out.ranges[count] = scan_in->ranges[i];



    count++;

    if(current_angle + scan_in->angle_increment > angle_abs)
    break;

    current_angle += scan_in->angle_increment;
    }
  }

  scan_out.header.frame_id = scan_in->header.frame_id;
  scan_out.header.stamp = start_time;
  scan_out.angle_min = -angle_abs;
  scan_out.angle_max = angle_abs;
  scan_out.angle_increment = scan_in->angle_increment;
  scan_out.time_increment = scan_in->time_increment;
  scan_out.scan_time = scan_in->scan_time;
  scan_out.range_min = scan_in->range_min;
  scan_out.range_max = scan_in->range_max;
  projector.projectLaser(scan_out,cloud);

} else {
  projector.projectLaser(*scan_in,cloud);
}




cloud.header.frame_id = cloud_frame_name;
cloud.header.stamp = ros::Time::now();
pub.publish(cloud);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lasertopc2");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string output_cloud_name;
  std::string scan_topic_name;

  if(!private_nh.getParam("angle_bound",angle))
   {
     angle = 0;
     ROS_WARN("Need to set parameter 'angle_bound' ");
   }

  if(!private_nh.getParam("scan_topic",scan_topic_name))
   {
     scan_topic_name = "scan";
     ROS_WARN("Need to set parameter 'scan_topic_name'.. set to \"%s\"",scan_topic_name.c_str());
   }
  if(!private_nh.getParam("output_cloud",output_cloud_name))
   {
     output_cloud_name = "cloud";
     ROS_WARN("Need to set parameter 'output_cloud'.. set to \"%s\"",output_cloud_name.c_str());
   }

   if(!private_nh.getParam("cloud_frame",cloud_frame_name)){
    cloud_frame_name = output_cloud_name;
    ROS_WARN("Need to set parameter 'cloud_frame'.. set to \"%s\"",cloud_frame_name.c_str());
   }

   ros::Subscriber sub = nh.subscribe(scan_topic_name,10,laser_callback);
    pub  = nh.advertise<sensor_msgs::PointCloud2>(output_cloud_name, 10);


  ros::spin();



  return 0;
}
