#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>

namespace gpar_nodelet {

    class listener : public nodelet::Nodelet {

        public:
        virtual void onInit(){
            NODELET_INFO("listerner constructor");
            ros::NodeHandle& nh = getNodeHandle();

            sub = nh.subscribe<std_msgs::String>("talker_topic",1,&listener::callback,this);
            
        }

        private:
        ros::Subscriber sub;
        void callback(const std_msgs::StringConstPtr& msg){
            NODELET_INFO("Listener got msg: %s",msg->data.c_str());
        }

    };
}


PLUGINLIB_EXPORT_CLASS(gpar_nodelet::listener, nodelet::Nodelet)