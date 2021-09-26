#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include "std_msgs/String.h"


namespace gpar_nodelet
{

    class talker : public nodelet::Nodelet
    {

    public:
        talker() {}

        virtual void onInit();



    private:

        void timerCb(const ros::TimerEvent& event);
        void intraCb(const std_msgs::StringConstPtr& msg);
        ros::Publisher pub;
        ros::Subscriber intra_sub;
        ros::Timer timer_;
        
    };

}