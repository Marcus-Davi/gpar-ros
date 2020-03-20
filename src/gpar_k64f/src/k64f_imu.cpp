/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>

ros::Publisher pub_a;
ros::Publisher pub_g;
ros::Publisher pub_m;
ros::Publisher pub_imu;

#define ACC_CONVERSION (0.488e-3 * 9.80665)
#define MAG_CONVERSION (0.1f)
#define GYR_CONVERSION (15.625e-3 * M_PI/180.0)


void serial_callback(const std_msgs::String::ConstPtr& msg){
int ax,ay,az;
int gx,gy,gz;
int mx,my,mz;

geometry_msgs::Vector3 acc,mag,gyr;
sensor_msgs::Imu imu;


// TODO tem forma melhor ?
sscanf(msg->data.c_str(),"%d %d %d %d %d %d %d %d %d",
  &ax,&ay,&az,
  &gx,&gy,&gz,
  &mx,&my,&mz);

acc.x = ax * ACC_CONVERSION;
acc.y = ay * ACC_CONVERSION;
acc.z = az * ACC_CONVERSION;

gyr.x = gx * GYR_CONVERSION;
gyr.y = gy * GYR_CONVERSION;
gyr.z = gz * GYR_CONVERSION;

mag.x = mx * MAG_CONVERSION;
mag.y = my * MAG_CONVERSION;
mag.z = mz * MAG_CONVERSION;


imu.header.stamp = ros::Time::now();
imu.header.frame_id = "k64f";

imu.angular_velocity.x = gyr.x;
imu.angular_velocity.y = gyr.y;
imu.angular_velocity.z = gyr.z;

imu.linear_acceleration.x = acc.x;
imu.linear_acceleration.y = acc.y;
imu.linear_acceleration.z = acc.z;

pub_a.publish(acc);
pub_g.publish(gyr);
pub_m.publish(mag);
pub_imu.publish(imu);



}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  ros::init(argc, argv, "gpar_k64f_imu");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle n("~");

  ros::Subscriber sub = n.subscribe("/mcuserial_node/serial_data", 1000, serial_callback);
  pub_a = n.advertise<geometry_msgs::Vector3>("accelerations",10);
  pub_g = n.advertise<geometry_msgs::Vector3>("angular_vels",10);
  pub_m = n.advertise<geometry_msgs::Vector3>("magnetic_field",10);
  pub_imu = n.advertise<sensor_msgs::Imu>("imu",10);


	ros::spin();
  return 0;
}





