/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <djiosdk/dji_vehicle.hpp>
#include "dji_sdk/MissionWpGetInfo.h"
#include "dji_sdk/MissionWaypointTask.h"
#include <dji_sdk/SendMobileData.h> //responde MSDK


#include "gpar_lidar/Command.h"


#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

// 1m ~ 0.000001 graus !
#define EPSILON 0.000038f //10m

bool CompareCoordinates(double a, double b);
bool HasReachedWaypoint(const sensor_msgs::NavSatFix& CurrentGPS, const dji_sdk::MissionWaypoint& Waypoint);
bool MobileSendText(const char * text, ros::ServiceClient& mobile_data_service);

bool isLogging = false;


ros::ServiceClient waypointInfo;
ros::ServiceClient cloudController;
ros::ServiceClient mobile_data_service;
ros::Publisher status_pub;

enum class WayPointState {
	None,
	OnWay,
	First,
	Last,
	Finished
};


void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg){
if(msg->data == 5){
	ROS_INFO("Pousando...");
	//system("rosnode kill /flight_bag");
}
	//	ros::shutdown(); //pousando

}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) //~50Hz
{
	sensor_msgs::NavSatFix current_gps = *msg;
	static std::string str_msg;
	static dji_sdk::MissionWaypointTask waypointTask;
	static dji_sdk::MissionWpGetInfo srv;
	static gpar_lidar::Command cloud_srv; //Call Cloud Controller
	std_msgs::UInt8 pub_msg;
	unsigned int Waypoints_Size;
    static WayPointState State = WayPointState::None;


	static unsigned int Waypoints_Index = 0; //

	if(!waypointInfo.call(srv)){ //Pergunta se há waypoints
		ROS_INFO("Esperando Mission . . .");
		pub_msg.data = -1;
		status_pub.publish(pub_msg);
	return;
	}

	if(isLogging == false){
	//	system("rosrun rosbag record -o /home/linaro/Logs/drone_scan /cloud_ldmrs /tf /tf_static __name:=flight_bag &");
		isLogging = true;
	}
	//Aqui ja tem waypoints!


	//Waypoints carregados aqui
	waypointTask = srv.response.waypoint_task;
	Waypoints_Size = waypointTask.mission_waypoint.size();


	if(Waypoints_Index <= Waypoints_Size){ //Ñ chegou ao final
	if(HasReachedWaypoint(current_gps,waypointTask.mission_waypoint[Waypoints_Index] ) == true){
		Waypoints_Index++;
		str_msg = "Chegou ao WP " + std::to_string(Waypoints_Index);
		MobileSendText(str_msg.c_str(),mobile_data_service);
	}
	}

	if(Waypoints_Index == 1) { //primeiro waypoint!
		if(State != WayPointState::First){
			cloud_srv.request.command = 1; //Start Scan
			if(!cloudController.call(cloud_srv))
		         ROS_ERROR("Falha ao chamar cloudcontroller service");
			else
				State = WayPointState::First;
		}

	} else if (Waypoints_Index == Waypoints_Size) { //Ultimo
		if(State != WayPointState::Last){
			cloud_srv.request.command = 0; //Stop Scan
			cloudController.call(cloud_srv);
			cloud_srv.request.command = 3; //Save Points
			if(!cloudController.call(cloud_srv))
				ROS_ERROR("Falha ao chamar cloudcontroller service");
			else
				State = WayPointState::Last;
	}

	}


		pub_msg.data = Waypoints_Index;
		status_pub.publish(pub_msg);





} //callback



int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoints_supervisor");
  ros::NodeHandle nh;
//  ros::NodeHandle nh_private("~");

  waypointInfo = nh.serviceClient<dji_sdk::MissionWpGetInfo>("dji_sdk/mission_waypoint_getInfo");

  cloudController = nh.serviceClient<gpar_lidar::Command>("cloud_controller/command_parser");

  mobile_data_service = nh.serviceClient<dji_sdk::SendMobileData>("dji_sdk/send_data_to_mobile");
 
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status",10,&flight_status_callback);

  ros::Subscriber gpsSub  = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback); //50Hz

  status_pub = nh.advertise<std_msgs::UInt8>("waypoint_supervisor", 100);


  ros::spin();

  return 0;

}



bool HasReachedWaypoint(const sensor_msgs::NavSatFix& CurrentGPS, const dji_sdk::MissionWaypoint& Waypoint){
	bool isLatEqual = CompareCoordinates(CurrentGPS.latitude,Waypoint.latitude);
	bool isLonEqual = CompareCoordinates(CurrentGPS.longitude,Waypoint.longitude);

	return (isLatEqual && isLonEqual);
}


bool CompareCoordinates(double a, double b){
	return fabs(a - b) < EPSILON;
}

	static dji_sdk::SendMobileData mobile_data_send;
bool MobileSendText(const char * text, ros::ServiceClient& mobile_data_service){

	std::string str_text(text);
	mobile_data_send.request.data.resize(str_text.size());
	memcpy(&mobile_data_send.request.data[0],str_text.c_str(),str_text.size());

	return mobile_data_service.call(mobile_data_send);

}



