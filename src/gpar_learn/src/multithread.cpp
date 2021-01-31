#include "ros/spinner.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>


void callback_a(const std_msgs::Float32ConstPtr& msg){
ROS_INFO("callback a : %f",msg->data);

}


void callback_b(const std_msgs::Float32ConstPtr& msg){
ROS_INFO("callback b : %f",msg->data);


}


int main(int argc,char** argv){
		ros::init(argc,argv,"multithread_test");
		ros::NodeHandle nh;


		ROS_DEBUG("D: multithead spinner test");
		ROS_INFO("I: multithead spinner test");
		ROS_WARN("W : multithead spinner test");
		ROS_ERROR("E : multithead spinner test");
		ROS_FATAL("F : multithead spinner test");

		ros::Subscriber sub_a = nh.subscribe("subscribe_a",10, callback_a);
		ros::Subscriber sub_b = nh.subscribe("subscribe_b",10, callback_b);

		ros::MultiThreadedSpinner spinner(4);
		spinner.spin();



		return 0;

}
