#include <pluginlib/class_list_macros.h>
#include "talker.h"

namespace gpar_nodelet {

    void talker::onInit(){
        NODELET_INFO("Talker Nodelet");
        ros::NodeHandle& nh = getMTNodeHandle();
        ros::NodeHandle& nh_ = getPrivateNodeHandle();
        

        timer_ = nh_.createTimer(ros::Duration(0.1),&talker::timerCb,this);
        pub = nh.advertise<std_msgs::String>("talker_topic",10);
        // intra_sub = nh.subscribe<std_msgs::String>("talker_topic",10,&talker::intraCb,this);


    }

    void talker::timerCb(const ros::TimerEvent& event){
        static int count = 0;

        std_msgs::StringPtr msg (new std_msgs::String);
        msg->data = "talker msg " + std::to_string(count++);
        pub.publish(msg); // pass pointer for zero copy
        // ros::Duration(0.05).sleep(); // tesing shared pointer change
        msg->data = "talker msg CHANGED!";
    }

    void talker::intraCb(const std_msgs::StringConstPtr& msg){
        ros::Duration(0.5).sleep(); // tesing shared pointer change
        NODELET_INFO("intracb: %s",msg->data.c_str());
    }


}


PLUGINLIB_EXPORT_CLASS(gpar_nodelet::talker, nodelet::Nodelet)