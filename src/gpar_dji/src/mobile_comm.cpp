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
#include <dji_sdk/MobileData.h>
#include <dji_sdk/SendMobileData.h>
#include <gpar_lidar/Command.h>


dji_sdk::MobileData mobile_data;
dji_sdk::SendMobileData mobile_data_send;
ros::ServiceClient client;
ros::ServiceClient mobile_data_service;


void mobile_comm_callback(const dji_sdk::MobileData::ConstPtr& from_mobile_data) // ~100Hz
{
  static char buffer[100];
  static gpar_lidar::Command srv;
  bool valid = false;


  mobile_data = *from_mobile_data;
  int msg_size = mobile_data.data.size();
  strncpy(buffer,(char*)&mobile_data.data[0],msg_size);
  buffer[msg_size] = 0;

  switch(buffer[0]){
  case 'T': //sTop
  srv.request.command = 0;
  valid = true;
  break;

  case 'I': //Init
  srv.request.command = 1;
  valid = true;
  break;

  case 'R': //Restart
  srv.request.command = 2;
  valid = true;
  break;

  case 'S': //Save
  srv.request.command = 3;
  valid = true;
  break;

  default:
  break;

  } //switch end

 if(valid){ //Comando valido ?
  if(client.call(srv)){ //Chama o serviço do cloud_controller
    if (mobile_data_service.call(mobile_data_send) ) //Envio OK ?
     ROS_INFO("Cmd recebido, OK enviado"); 
  } 
  else
  ROS_INFO("srv call bad");
}


  

  

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mobile_comm");
  ros::NodeHandle nh;

// Subscribe to messages from dji_sdk_node

   ros::Subscriber mobile_comm_sub = nh.subscribe("dji_sdk/from_mobile_data", 10, &mobile_comm_callback);
   mobile_data_service = nh.serviceClient<dji_sdk::SendMobileData>("dji_sdk/send_data_to_mobile");
   client = nh.serviceClient<gpar_lidar::Command>("cloud_controller/command_parser");


   //Copy OK to SendData package
   std::string AckStr("OK");
   mobile_data_send.request.data.resize(AckStr.size());
   memcpy(&mobile_data_send.request.data[0],AckStr.c_str(),AckStr.size());




  ros::spin(); //Chama os callbacks
  return 0;
}









