/* Author : Marcus Forte <davi2812@dee.ufc.br> */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose.h>
#include <pcl/io/pcd_io.h>

#define LOW_LIMIT 80

ros::Publisher pub_;

static nav_msgs::OccupancyGrid::Ptr map_g = boost::make_shared<nav_msgs::OccupancyGrid>();
static sensor_msgs::PointCloud2::Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();



//Usar message filter!!
void map_callback(const nav_msgs::OccupancyGrid::Ptr map){
//  ROS_INFO("GOT MAP!");
map_g = map;

}


void save_map_callback(const std_msgs::String::ConstPtr& msg){
  if(msg->data == "save"){
//ROS_INFO("Saving MAP!");
  //Faz varredura do mapar
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
unsigned int  width = map_g->info.width; // largura
unsigned int  height = map_g->info.height; //altura
  float resolution = map_g->info.resolution;
unsigned int size = width*height;
  float x,y; //Posição da celular
  uint32_t row,column;
  geometry_msgs::Pose p0 = map_g->info.origin;

  ROS_INFO("frame = %s, cells = %d",map_g->header.frame_id.c_str(),size);

  unsigned int points = 0;
  for(unsigned int i=0;i<size;++i){
	if(map_g->data[i] > LOW_LIMIT){ //verifica se ponto eh valido
	points++;
	column = i%height;
	row = i/width;
//	ROS_INFO("i = %d, row = %d, column = %d",i,row,column);
	y = row*resolution;
	x = column*resolution;
	x = x + p0.position.x;
	y = y + p0.position.y;
//	ROS_INFO("x = %f, y = %f",x,y);
	cloud->push_back(pcl::PointXYZ (x,y,0));
	  }
  }

  if(points){
	pcl::io::savePCDFileBinary("/home/projeto/MAPA.pcd",*cloud);
	ROS_INFO("Map Saved! Points = %lu",cloud->points.size());
	pcl::toROSMsg(*cloud,*cloud_msg);
	cloud_msg->header.stamp = ros::Time::now();
	cloud_msg->header.frame_id= "map";
	pub_.publish(cloud_msg);
  }

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
  ros::NodeHandle nh("~");

  if (nh.getParam("map_topic",map_topic))
  ROS_INFO("parametro 'serial_port' lido com sucesso : %s",map_topic.c_str());
   else {
  ROS_INFO("parametro 'serial_port' n existe! setado : %s",map_topic.c_str());
  }

  ros::Subscriber sub_command = n.subscribe("save_map",100,save_map_callback);
  ros::Subscriber sub_map = n.subscribe(map_topic,10,map_callback);

  pub_ = n.advertise<sensor_msgs::PointCloud2>("map_cloud",10);


  ros::spin();


  return 0;
}
