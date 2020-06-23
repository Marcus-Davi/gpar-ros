#include "message_filters/synchronizer.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <gpar_learn/FloatStamped.h>



gpar_learn::FloatStamped global1;
gpar_learn::FloatStamped global2;


void t1_callback(const gpar_learn::FloatStamped::ConstPtr& msg){
		ROS_INFO("t1_callback. value : %f",msg->data);
		global1 = *msg;
}

void t2_callback(const gpar_learn::FloatStamped::ConstPtr& msg){
		ROS_INFO("t2_callback. value : %f",msg->data);
		global2 = *msg;
}

void unsynced_callback(const gpar_learn::FloatStampedConstPtr& msg){
		ROS_WARN("unsynced callback. sum = (m1:%f + m2:%f) = %f",global1.data,global2.data, global1.data + global2.data);
}


void sync_callback(const gpar_learn::FloatStamped::ConstPtr& msg1,const gpar_learn::FloatStamped::ConstPtr& msg2){
		ROS_WARN("apprimate time callback. sum = (m1:%f + m2:%f) = %f",msg1->data,msg2->data, msg1->data + msg2->data);
}


void time_sync_callback(const gpar_learn::FloatStamped::ConstPtr& msg1,const gpar_learn::FloatStamped::ConstPtr& msg2){
		ROS_WARN("time_sync callback. sum = (m1:%f + m2:%f) = %f",msg1->data,msg2->data, msg1->data + msg2->data);
}


int main(int argc,char** argv){
		ros::init(argc,argv,"msg_filter_test_node");

		ros::NodeHandle nh;
		

		std::string topic1 = nh.resolveName("topico1");
		std::string topic2 = nh.resolveName("topico2");
		//
		//std::string topic1 = "topico1";
		//std::string topic2 = "topico2";

		ROS_INFO("topic2 -> %s",topic1.c_str());
		ROS_INFO("topic2 -> %s",topic2.c_str());

		ros::Subscriber sub1 = nh.subscribe(topic1,10,t1_callback);
		ros::Subscriber sub2 = nh.subscribe(topic2,10,t2_callback);
		ros::Subscriber unsync_sub = nh.subscribe(topic2,10,unsynced_callback);

		// Conventional Sub



		// Msg Filter test
		typedef message_filters::sync_policies::ApproximateTime<gpar_learn::FloatStamped, gpar_learn::FloatStamped> MySyncPolicy;	


		message_filters::Subscriber<gpar_learn::FloatStamped> s_sub1(nh,topic1,1);
		message_filters::Subscriber<gpar_learn::FloatStamped> s_sub2(nh,topic2,1);

		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),s_sub1,s_sub2);

		sync.registerCallback(boost::bind(&sync_callback,_1,_2));
		//sync.registerCallback(sync_callback);

	
		message_filters::TimeSynchronizer<gpar_learn::FloatStamped,gpar_learn::FloatStamped> sync2(s_sub2,s_sub1,10);
		//sync.registerCallback(boost::bind(&time_sync_callback,_1,_2));
		sync.registerCallback(time_sync_callback);




		ros::spin();

}
