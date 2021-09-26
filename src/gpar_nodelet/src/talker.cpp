#include <pluginlib/class_list_macros.h>
#include "talker.h"
#include "std_msgs/String.h"
namespace gpar_nodelet {

    void talker::onInit(){
        NODELET_INFO("Talker Nodelet");
        ros::NodeHandle& nh = getNodeHandle();
        ros::NodeHandle& nh_ = getPrivateNodeHandle();

        timer_ = nh_.createTimer(ros::Duration(0.1),&talker::timerCb,this);
        pub = nh.advertise<std_msgs::String>("talker_topic",5);


    }

    void talker::timerCb(const ros::TimerEvent& event){
        static int count = 0;

        std_msgs::String msg;
        msg.data = "talker msg " + std::to_string(count++);
        pub.publish(msg);
    }


}


PLUGINLIB_EXPORT_CLASS(gpar_nodelet::talker, nodelet::Nodelet)