#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <sensor_msgs/PointCloud2.h>

static nav_msgs::OccupancyGrid map_g;
static sensor_msgs::PointCloud2 cloud;


void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& map){
  ROS_INFO("GOT MAP!");
map_g = *map;




}


void save_map_callback(const std_msgs::String::ConstPtr& msg){
  if(msg->data == "save"){
ROS_INFO("Saving MAP!");
  ROS_INFO("frame = %s",map_g.header.frame_id.c_str());




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
  ros::init(argc, argv, "grid_map_save_node");
  std::string map_topic = "map";


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  if (n.getParam("map_topic",map_topic))
  ROS_INFO("parametro 'serial_port' lido com sucesso : %s",map_topic.c_str());
   else {
  ROS_INFO("parametro 'serial_port' n existe! setado : %s",map_topic.c_str());
  }

  ros::Subscriber sub_command = n.subscribe("save_map",100,save_map_callback);
  ros::Subscriber sub_map = n.subscribe(map_topic,10,map_callback);




  ros::spin();


  return 0;
}
