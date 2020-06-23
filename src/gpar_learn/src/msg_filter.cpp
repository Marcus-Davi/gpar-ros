#include "message_filters/synchronizer.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


void t1_callback(const geometry_msgs::PointStamped::ConstPtr& msg){
		ROS_INFO("t1_callback");
}

void t2_callback(const geometry_msgs::PointStamped::ConstPtr& msg){
		ROS_INFO("t2_callback");

}


void sync_callback(const geometry_msgs::PointStamped::ConstPtr& msg1,const geometry_msgs::PointStamped::ConstPtr& msg2){
		ROS_INFO("sync callback");


}




int main(int argc,char** argv){
		ros::init(argc,argv,"msg_filter_test_node");

		ros::NodeHandle nh;

		//std::string topic1 = nh.resolveName("topic1");
		//std::string topic2 = nh.resolveName("topic2");
		//
		std::string topic1 = "topico1";
		std::string topic2 = "topico2";

		ROS_INFO("topics -> %s",topic1.c_str());
		ros::Subscriber sub1 = nh.subscribe(topic1,10,t1_callback);
		ros::Subscriber sub2 = nh.subscribe(topic2,10,t2_callback);


		// Msg Filter test
		typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, geometry_msgs::PointStamped> MySyncPolicy;	


		message_filters::Subscriber<geometry_msgs::PointStamped> s_sub1(nh,topic1,1);
		message_filters::Subscriber<geometry_msgs::PointStamped> s_sub2(nh,topic2,1);

		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),s_sub1,s_sub2);

		//message_filters::TimeSynchronizer<geometry_msgs::PointStamped,geometry_msgs::PointStamped> sync(s_sub1,s_sub2,10);
		//sync.registerCallback(sync_callback);
		sync.registerCallback(boost::bind(&sync_callback,_1,_2));





		ros::spin();

}
