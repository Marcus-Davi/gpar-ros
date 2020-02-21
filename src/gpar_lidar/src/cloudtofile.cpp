#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

static sensor_msgs::PointCloud2::Ptr cloud_g = boost::make_shared<sensor_msgs::PointCloud2>();
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudT;



void cloud_callback(const sensor_msgs::PointCloud2::Ptr cloud){
  ROS_INFO("GOT CLOUD!");
cloud_g = cloud;

}


void save_map_callback(const std_msgs::String::ConstPtr& msg){
  static PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();
  if(msg->data == "save"){

    pcl::fromROSMsg(*cloud_g, *cloud);
    ros::Time time = ros::Time::now();
    std::time_t std_time;
    std_time = time.sec;
    char* time_date = std::ctime(&std_time);
    char* home_path = getenv("HOME");
    std::string str_path = std::string(home_path) + "/Pontos/pontos_sweep_";
    //std::string str_time = std::string(time_date);
    std::string str_time = std::to_string(time.sec);
    std::string str_pcd = str_path + str_time + ".pcd";
    pcl::io::savePCDFileBinary(str_pcd,*cloud);
    ROS_INFO("PCD Saved!");
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



  ros::init(argc, argv, "pc_save_file");
	if (argc < 2 ) {

		ROS_ERROR("Uso : Insira o topic para captura de nuvem de pontos");
		return 1;
		
	} 


  	std::string cloud_topic = argv[1];
	ROS_INFO("inscrito no topico %s",argv[1]);

  bool has_color = false;


  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::NodeHandle nh("~");



  ros::Subscriber sub_command = n.subscribe("save_cloud",100,save_map_callback);
  ros::Subscriber sub_cloud = n.subscribe(cloud_topic,10,cloud_callback);


  ros::spin();


  return 0;
}
