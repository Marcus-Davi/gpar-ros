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

#include "gpar_dji/telemetry.h"
#include <fstream>

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

//ros::ServiceClient set_local_pos_reference;
//ros::ServiceClient sdk_ctrl_authority_service;
//ros::ServiceClient drone_task_service;
//ros::ServiceClient query_version_service;

//ros::Publisher ctrlPosYawPub;
//ros::Publisher ctrlBrakePub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;


sensor_msgs::NavSatFix current_gps;
sensor_msgs::NavSatFix origin_gps;

geometry_msgs::Quaternion current_atti; //global

bool InitFlag = false;

//ros::Publisher PosePub;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "telemetry");
  ros::NodeHandle nh;

// Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback); //100Hz
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback); //50Hz
//  PosePub = nh.advertise<geometry_msgs::Pose>("telemetry_pose",10);


 
  //ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  //ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  //ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

	//ROS_INFO("q0 = %f,q1 = %f,q2 = %f,q3 = %f",current_atti.w,current_atti.x,current_atti.y,current_atti.z);
	//ROS_INFO("q0 = %f,q1 = %f,q2 = %f,q3 = %f",current_atti.w,current_atti.x,current_atti.y,current_atti.z);



  ros::spin(); //Chama os callbacks
  return 0;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates. Accurate when distances are small.
!*/

/*
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                         sensor_msgs::NavSatFix& target,
                         sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}


bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}*/

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) // ~100Hz
{
  current_atti = msg->quaternion;

static geometry_msgs::Vector3 Position; //Position

static tf2_ros::TransformBroadcaster br; //tf Broadcaster
geometry_msgs::TransformStamped transformStamped;

transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = "map";
transformStamped.child_frame_id = "drone";

static bool InitFlag = false;

if(InitFlag == false){
	if(current_gps.latitude == 0.0f){
//	ROS_INFO("INVALID GPS COORDINATE");
	Position.x = 0;
	Position.y = 0;
	Position.z = 0;
	} else {
//	origin_gps = current_gps;
	//podemos fixar as coordenadas aqui!
	origin_gps.latitude = -3.584132; //coordenadas centrais das pilhas
  	origin_gps.longitude = -38.879044; //coordenadas centrais das pilhas

	ROS_INFO("FIRST COORDINATE MEMORIZED !");
	InitFlag = true;
	}
		} else { //GPS ORIGIN SET!
  double deltaLat = current_gps.latitude - origin_gps.latitude;
  double deltaLon = current_gps.longitude - origin_gps.longitude;

  Position.y = deltaLat * deg2rad * C_EARTH;
  Position.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*current_gps.latitude);
  Position.z = current_gps.altitude;
  

//  ROS_INFO("x = %f,y = %f",Position.x,Position.y);
		}

//	geometry_msgs::Vector3 ans;
//  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(current_atti.x, current_atti.y, current_atti.z, current_atti.w));
//  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  // ROS_INFO("roll = %f,pitch = %f, yaw = %f",ans.x, ans.y, ans.z);

transformStamped.transform.translation.x = Position.x;
transformStamped.transform.translation.y = Position.y;
transformStamped.transform.translation.z = Position.z;

transformStamped.transform.rotation = current_atti;

br.sendTransform(transformStamped); //Always Publish

}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) //~50Hz
{
current_gps = *msg;
}




