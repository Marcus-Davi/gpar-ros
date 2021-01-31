#include <ros/ros.h>
#include <std_msgs/String.h>
#include "gpar_datalog/Command.h"


// my lib
#include "gpar_datalog/datalog.h"

int main(int argc,char** argv){

		ros::init(argc,argv,"bag_generator");

		ros::NodeHandle nh;
		ros::NodeHandle nh_private("~");


		// TODO processar 'argv' p/ coletar os topicos utilizados
		


		std::string filename;

		std::string home = std::string(getenv("HOME"));

		if(!nh_private.getParam("bag_filename",filename)){
				filename = home + "/test_bag";
				ROS_WARN("please set 'bag_filename' parameter. default filename set -> %s",filename.c_str());
		} else {
				ROS_INFO("bag save path set to -> %s",filename.c_str());
		}


		

		filename = filename + "_" + std::to_string((int)(ros::Time::now().toSec())) + ".bag";


		// usar classe !


		gpar::datalog a;






		ros::spin();

}
