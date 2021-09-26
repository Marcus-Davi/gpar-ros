#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>


namespace gpar_nodelet
{

    class talker : public nodelet::Nodelet
    {

    public:
        talker() {}

        virtual void onInit();

    private:

        void timerCb(const ros::TimerEvent& event);
        ros::Publisher pub;
        ros::Timer timer_;
        
    };

}