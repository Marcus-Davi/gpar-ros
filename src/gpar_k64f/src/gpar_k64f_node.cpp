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
#include <geometry_msgs/QuaternionStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>

std::string child_frame;
std::string frame;


void serial_callback(const std_msgs::String::ConstPtr& msg){
static geometry_msgs::Quaternion attitude; //quaternion
//Transform data
static tf2_ros::TransformBroadcaster br; //tf Broadcaster

geometry_msgs::TransformStamped transformStamped;
transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = frame; //"map" ou "odom"
transformStamped.child_frame_id = child_frame; //"cloud" ou "imu"

transformStamped.transform.translation.x = 0;
transformStamped.transform.translation.y = 0;
transformStamped.transform.translation.z = 1; 


	float q0,q1,q2,q3;
	sscanf(msg->data.c_str(),"%f %f %f %f",&q0,&q1,&q2,&q3);
float norm = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);

attitude.w = q0/norm;
attitude.x = q1/norm;
attitude.y = q2/norm;
attitude.z = q3/norm;


transformStamped.transform.rotation=attitude;

br.sendTransform(transformStamped);

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

  if ( n.getParam("k64f_child_frame",child_frame) )
{}
   else{
	ROS_INFO("k64f_child_frame NOT SET");
	child_frame = "k64f_frame_child";
	}

  if ( n.getParam("k64f_frame",frame) )
{}
   else{
	ROS_INFO("k64f_frame NOT SET");
	frame = "k64f_frame";
	}

	ROS_INFO("k64f_frame : %s",frame.c_str());
	ROS_INFO("k64f_child_frame : %s",child_frame.c_str());

  ros::Subscriber sub = n.subscribe("/mcuserial_node/serial_data", 1000, serial_callback);


	ros::spin();
  return 0;
}





