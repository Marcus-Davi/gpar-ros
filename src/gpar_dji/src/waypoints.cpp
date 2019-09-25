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

#include "gpar_dji/waypoints.h"
#include "gpar_dji/waypoints_loader.h"


ros::ServiceClient waypoint_upload_service;
ros::ServiceClient waypoint_action_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient mobile_data_service;
ros::ServiceClient waypoint_setspeed_service;
ros::Subscriber mobile_comm_sub;

dji_sdk::SendMobileData mobile_data_send;

dji_sdk::MissionWaypointTask waypointTask;
bool startMission = false;
float idle_vel = 8;

std::string str_waypointfile;


//test

void mobile_comm_callback(const dji_sdk::MobileData::ConstPtr& from_mobile_data){
  static char buffer[100];
   static dji_sdk::MissionWpSetSpeed srv;
  bool valid = false;


  dji_sdk::MobileData mobile_data = *from_mobile_data;
  int msg_size = mobile_data.data.size();
  strncpy(buffer,(char*)&mobile_data.data[0],msg_size);
  buffer[msg_size] = 0;

    if(buffer[0]=='V') //VELOCITY
    {
    idle_vel = atof(&buffer[2]);
    srv.request.speed = idle_vel;
    if(waypoint_setspeed_service.call(srv))
    ROS_INFO("cruising speed = %f",idle_vel);
    MobileSendText("Velocidade Setada com sucesso",mobile_data_service);
    valid = true;
    }
    else if(buffer[0]=='W') //WAYPOINT!
    {
	char* home_path = getenv("HOME");
	std::string str_waypointpath = std::string(home_path) + std::string("/Waypoints/");
	std::string str_waypointfilename;
   switch(buffer[2]){
   case '0': //W=0
   valid = true;
   MobileSendText("Missao Campo Carregada",mobile_data_service);
   startMission = true;
   str_waypointfilename = "campo.wpt";
   str_waypointfile = str_waypointpath + str_waypointfilename;
   break;

   case '1': //W=1
   valid = true;
   MobileSendText("Missao Campo Pilha 1",mobile_data_service);
   str_waypointfilename = "pilha1.wpt";
   startMission = true;
   str_waypointfile = str_waypointpath + str_waypointfilename;
   break;

   case '2': //W=2
   valid = true;
   MobileSendText("Missao Campo Pilha 2",mobile_data_service);
   str_waypointfilename = "pilha2.wpt";
   startMission = true;
   str_waypointfile = str_waypointpath + str_waypointfilename;
   break;

   case '3': //W=3
   valid = true;
   MobileSendText("Missao Campo Pilha 3",mobile_data_service);
   str_waypointfilename = "pilha3.wpt";
   startMission = true;
   str_waypointfile = str_waypointpath + str_waypointfilename;
   break;

   default:
   break;
   }

    }

    if(valid)
    { //Comando valido ?

    } 

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoints");
  ros::NodeHandle nh;
  //ros::NodeHandle nh_private("~");

  std::vector<std::string> v_string;
  bool dji_ok;

  waypoint_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>("dji_sdk/mission_waypoint_upload");

  waypoint_action_service = nh.serviceClient<dji_sdk::MissionWpAction>("dji_sdk/mission_waypoint_action");

  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");

  drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");

  mobile_data_service = nh.serviceClient<dji_sdk::SendMobileData>("dji_sdk/send_data_to_mobile");

  waypoint_setspeed_service = nh.serviceClient<dji_sdk::MissionWpSetSpeed>("dji_sdk/mission_waypoint_setSpeed");

  mobile_comm_sub  = nh.subscribe("dji_sdk/from_mobile_data", 10, &mobile_comm_callback);

  ros::Rate r(10);
  while(ros::ok()){
  
  ros::spinOnce();

   r.sleep();

  if(!startMission)
   continue;

   ServiceAck ack = obtainCtrlAuthority(); //DEBUG


  if (ack.result)
  {
    ROS_INFO("Obtain SDK control Authority successfully");
  }
  else
  {
    if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0)
    {
      ROS_INFO("Obtain SDK control Authority in progess, "
               "send the cmd again");
      obtainCtrlAuthority();
    }
    else
    {
      ROS_WARN("Failed Obtain SDK control Authority. Releasing. Try again!");
      releaseCtrlAuthority();
      startMission=false;
     system("rosnode kill dji_sdk"); //Gambiarra
     dji_ok = false;

     while(dji_ok == false){
    ros::master::getNodes(v_string);
    for(std::vector<std::string>::const_iterator i = v_string.begin(); i < v_string.end(); ++i){
    	ROS_INFO("node = %s",i->c_str());
    }
    ros::Duration(1).sleep();



     }



      continue;

    }
  }


   setWaypointInitDefaults(waypointTask);
   ROS_INFO("Waypoint loading : %s",str_waypointfile.c_str());
   std::vector<DJI::OSDK::WayPointSettings>* wp_list = WaypointLoad(str_waypointfile.c_str());
   if(wp_list == NULL){
   ROS_FATAL("Erro ao carregar arquivos waypoints");
   } else {
   ROS_INFO("Arquivo de Waypoint lido com sucesso!");
   }

  dji_sdk::MissionWaypoint waypoint; //Aqui eh definição do ROS!
   waypointTask.mission_waypoint.clear();
  for (std::vector<WayPointSettings>::iterator wp = wp_list->begin(); wp != wp_list->end(); ++wp)
  {
    ROS_INFO("Waypoint criado em (LLA): %f %f %f %d\n ", wp->latitude,wp->longitude, wp->altitude,wp->yaw);
    waypoint.latitude            = wp->latitude;
    waypoint.longitude           = wp->longitude;
    waypoint.altitude            = wp->altitude;
    waypoint.damping_distance    = 0;
    waypoint.target_yaw          = wp->yaw;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode           = 0;
    waypoint.has_action          = 0;
    waypointTask.mission_waypoint.push_back(waypoint);
  }

  ROS_INFO("Initializing Waypoint Mission..\n");
  if (initWaypointMission(waypointTask).result)
  {
    ROS_INFO("Waypoint upload command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending waypoint upload command");
  }

  // Waypoint Mission: Start
  if (missionAction(DJI_MISSION_TYPE::WAYPOINT,MISSION_ACTION::START).result)
  {
    ROS_INFO("Mission start command sent successfully");
  }
  else
  {
    ROS_WARN("Failed sending mission start command");
  }

  startMission=false;
}

}






ServiceAck takeoff()
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 4;
  drone_task_service.call(droneTaskControl);
  if (!droneTaskControl.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
             droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return ServiceAck(
    droneTaskControl.response.result, droneTaskControl.response.cmd_set,
    droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}

ServiceAck land()
{
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 6;
  drone_task_service.call(droneTaskControl);
  if (!droneTaskControl.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
             droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return ServiceAck(
    droneTaskControl.response.result, droneTaskControl.response.cmd_set,
    droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}



void setWaypointInitDefaults(dji_sdk::MissionWaypointTask& waypointTask)
{
  waypointTask.velocity_range     = 10;
  waypointTask.idle_velocity      = idle_vel; //5
  waypointTask.action_on_finish   = dji_sdk::MissionWaypointTask::FINISH_RETURN_TO_HOME; //FINISH_RETURN_TO_POINT FINISH_NO_ACTION
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode           = dji_sdk::MissionWaypointTask::YAW_MODE_WAYPOINT; //YAW_MODE_LOCK  YAW_MODE_AUTO YAW_MODE_WAYPOINT
  waypointTask.trace_mode         = dji_sdk::MissionWaypointTask::TRACE_POINT;
  waypointTask.action_on_rc_lost  = dji_sdk::MissionWaypointTask::ACTION_AUTO;
  waypointTask.gimbal_pitch_mode  = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}


ServiceAck obtainCtrlAuthority()
{
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 1;
  sdk_ctrl_authority_service.call(sdkAuthority);
  if (!sdkAuthority.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
             sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set,
                    sdkAuthority.response.cmd_id,
                    sdkAuthority.response.ack_data);
}

ServiceAck releaseCtrlAuthority()
{
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 0;
  sdk_ctrl_authority_service.call(sdkAuthority);
  if (!sdkAuthority.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
             sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set,
                    sdkAuthority.response.cmd_id,
                    sdkAuthority.response.ack_data);
}


ServiceAck initWaypointMission(dji_sdk::MissionWaypointTask& waypointTask)
{
  dji_sdk::MissionWpUpload missionWpUpload;
  missionWpUpload.request.waypoint_task = waypointTask;
  waypoint_upload_service.call(missionWpUpload);
  if (!missionWpUpload.response.result)
  {
    ROS_WARN("ack.info: set = %i id = %i", missionWpUpload.response.cmd_set,
             missionWpUpload.response.cmd_id);
    ROS_WARN("ack.data: %i", missionWpUpload.response.ack_data);
  }
  return ServiceAck(
    missionWpUpload.response.result, missionWpUpload.response.cmd_set,
    missionWpUpload.response.cmd_id, missionWpUpload.response.ack_data);
}

ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type, DJI::OSDK::MISSION_ACTION   action)
{
  dji_sdk::MissionWpAction missionWpAction;
  switch (type)
  {
    case DJI::OSDK::WAYPOINT:
      missionWpAction.request.action = action;
      waypoint_action_service.call(missionWpAction);
      if (!missionWpAction.response.result)
      {
        ROS_WARN("ack.info: set = %i id = %i", missionWpAction.response.cmd_set,
                 missionWpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionWpAction.response.ack_data);
      }
      return { missionWpAction.response.result,
               missionWpAction.response.cmd_set,
               missionWpAction.response.cmd_id,
               missionWpAction.response.ack_data };
  }
}


bool MobileSendText(const char * text, ros::ServiceClient& mobile_data_service){
	static dji_sdk::SendMobileData mobile_data_send;

	std::string str_text(text);
	mobile_data_send.request.data.resize(str_text.size());
	memcpy(&mobile_data_send.request.data[0],str_text.c_str(),str_text.size());

	return mobile_data_service.call(mobile_data_send);

}


// COMO ???
//[ WARN] [1569436644.239348600]: ack.info: set = 1 id = 0
//[ WARN] [1569436644.239551892]: ack.data: 2
//[ WARN] [1569436644.240054142]: Failed Obtain SDK control Authority. Releasing. Try again!







