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

#include "ros/ros.h"
#include "dji_sdk/MissionWpGetInfo.h"
#include "dji_sdk/MissionWaypointTask.h"
#include "gpar_lidar/Command.h"
#include <sensor_msgs/NavSatFix.h>
#include "std_msgs/UInt8.h"


// 1m ~ 0.000001 graus !
#define EPSILON 0.00002f //10m

ros::ServiceClient waypointInfo;
ros::ServiceClient cloudController;
ros::Publisher status_pub;


enum WayptStatus {
    Not_Ready = -1,
	No_Waypoint = 0,
	First_Waypoint = 1,
	Last_Waypoint = 2,
};

enum CloudStatus_ {
     Idle,
     Merging,
     Saving,
     Done
};


bool CompareCoordinates(double a, double b){
	return fabs(a - b) < EPSILON;
}


void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg){
if(msg->data == 5)
	ros::shutdown();

}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) //~50Hz
{
static sensor_msgs::NavSatFix current_gps;
static dji_sdk::MissionWaypointTask waypointTask;
static dji_sdk::MissionWpGetInfo srv;
static gpar_lidar::Command cloud_srv; //Call Cloud Controller
static CloudStatus_ CloudStatus = Idle;
std_msgs::UInt8 pub_msg;
pub_msg.data = Not_Ready;

current_gps = *msg; //Get GPS DATA

 if(! waypointInfo.call(srv))
  {
  ROS_INFO("AINDA N TEMOS WAYPOINTS");
  } else { //Waypoints Carregados!
  bool LatEqual = false;
  bool LonEqual = false;
  unsigned int size = waypointTask.mission_waypoint.size();
  waypointTask = srv.response.waypoint_task;

  LatEqual = CompareCoordinates(current_gps.latitude,waypointTask.mission_waypoint[0].latitude); //WP1
  LonEqual = CompareCoordinates(current_gps.longitude,waypointTask.mission_waypoint[0].longitude); //WP1

	if(LatEqual && LonEqual){
	 pub_msg.data = First_Waypoint; //0
         if(CloudStatus != Merging){
         cloud_srv.request.command = 1;
	if(!cloudController.call(cloud_srv))
         ROS_ERROR("Falha ao chamar cloudcontroller service");
         else
	 CloudStatus = Merging;
         } //if cloud != merging
     	 } else {
	 LatEqual = CompareCoordinates(current_gps.latitude,waypointTask.mission_waypoint[size-1].latitude); //WPN
	 LonEqual = CompareCoordinates(current_gps.longitude,waypointTask.mission_waypoint[size-1].longitude); //WPN
	 if(LatEqual && LonEqual){
	 pub_msg.data = Last_Waypoint; //2
         if(CloudStatus != Saving){
         cloud_srv.request.command = 0;
	 cloudController.call(cloud_srv);
         cloud_srv.request.command = 3;
          if(!cloudController.call(cloud_srv))
         ROS_ERROR("Falha ao chamar cloudcontroller service");
         else
	 CloudStatus = Saving;
	 } else { //latlong != final
	 pub_msg.data = No_Waypoint; //0
	 }

	  } //LatLong Equal if
	 }
   } //Else Waypoint carregados



   status_pub.publish(pub_msg);

} //callback



int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoints_supervisor");
  ros::NodeHandle nh;
//  ros::NodeHandle nh_private("~");

  waypointInfo = nh.serviceClient<dji_sdk::MissionWpGetInfo>("dji_sdk/mission_waypoint_getInfo");

  cloudController = nh.serviceClient<gpar_lidar::Command>("cloud_controller/command_parser");
 
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status",10,&flight_status_callback);

  ros::Subscriber gpsSub  = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback); //50Hz

  status_pub = nh.advertise<std_msgs::UInt8>("waypoint_supervisor", 1000);

  ros::spin();

  return 0;

}

