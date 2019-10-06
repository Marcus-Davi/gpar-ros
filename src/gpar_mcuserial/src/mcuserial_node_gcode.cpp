/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <serial/serial.h>

#include <thread>

std::string from_terminal;
bool terminal_ok = false;

void GetLineFromCin() {

	while(ros::ok()){

    std::getline(std::cin, from_terminal); //recebe do terminal
    std::cout << "msg do terminal : " << from_terminal << std::endl;
    terminal_ok = true;
	}
    
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */


  ros::init(argc, argv, "mcuserial_node_gcode");


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle n("~");
  std::string porta_serial;
  if ( n.getParam("serial_port",porta_serial) )
	ROS_INFO("parametro lido com sucesso : %s",porta_serial.c_str());
   else{
	porta_serial = "/dev/ttyUSB0";
	ROS_INFO("Porta n existe! usando padrao %s",porta_serial.c_str());
	}

  
  ros::Publisher pub = n.advertise<std_msgs::String>("serial_data", 1000);
  int count = 0;

  serial::Serial mcu_serial(porta_serial,115200,serial::Timeout::simpleTimeout(1000));

	//ASYNC
	std::thread input_read_thread(GetLineFromCin);
//	auto future = std::async(std::launch::async, GetLineFromCin);




if(mcu_serial.isOpen()){
 ROS_INFO("Porta Serial aberta!");
} else {
 ROS_INFO("Problema ao abrir a porta %s ! ela existe?",porta_serial.c_str());
return -1; //tentar novamente ?
}

  ros::Rate r(20);
 std_msgs::String leitura;
 std::string read_answer;

  while(ros::ok()){ //sai se der ctol_c

//comentado no laptop dell
if(terminal_ok){
	
	mcu_serial.write(from_terminal + "\r");
	read_answer = mcu_serial.readline(100,"\r");
	std::cout << "Slider : "<< read_answer << std::endl;
	terminal_ok = false;
        } else {

	mcu_serial.write("?");
	leitura.data = mcu_serial.readline(100,"\r");
	//ROS_INFO("%s",leitura.data.c_str());
	pub.publish(leitura);

}

	ros::spinOnce();
	r.sleep();

	}

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */



  return 0;
}


