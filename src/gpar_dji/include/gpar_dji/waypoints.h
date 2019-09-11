

#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <ros/ros.h>

#include <dji_sdk/MissionWpAction.h>
//#include <dji_sdk/MissionHpAction.h>
#include <dji_sdk/MissionWpUpload.h>
//#include <dji_sdk/MissionHpUpload.h>

#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>

#include <dji_sdk/MobileData.h> //recebe MSDK
#include <dji_sdk/SendMobileData.h> //responde MSDK
#include <dji_sdk/MissionWpSetSpeed.h> //muda Vel
#include <djiosdk/dji_vehicle.hpp>

#include <fstream>



typedef struct ServiceAck
{
  bool         result;
  int          cmd_set;
  int          cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack)
    : result(res)
    , cmd_set(set)
    , cmd_id(id)
    , ack_data(ack)
  {
  }
  ServiceAck()
  {
  }
} ServiceAck;

void setWaypointDefaults(WayPointSettings* wp);
void setWaypointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask);
ServiceAck takeoff();
ServiceAck land();
ServiceAck obtainCtrlAuthority();
ServiceAck initWaypointMission(dji_sdk::MissionWaypointTask& waypointTask);
ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type, DJI::OSDK::MISSION_ACTION   action);
bool MobileSendText(const char * text, ros::ServiceClient& mobile_data_service);

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))









#endif // WAYPOINTS_H
