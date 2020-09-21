/* Author : Marcus Forte <davi2812@dee.ufc.br> */

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <message_filters/subscriber.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>   // necessary because of custom point type
#include <pcl/filters/passthrough.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <std_msgs/UInt8.h>

tf2_ros::TransformListener *tf_listener;
tf2_ros::Buffer *tf_buffer;



typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

bool start_merge = false;
float x_min;
float x_max;

// Nuvem Global
PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();
ros::Publisher merge_signal;

std::string profiler_frame = "profiler_obj"; // Default profiler frame
float lidar_height;


// Callback
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr pc){


boost::shared_ptr<PointCloudT> cloud = boost::make_shared<PointCloudT>();
boost::shared_ptr<PointCloudT> cloud_filtered = boost::make_shared<PointCloudT>();

pcl::fromROSMsg(*pc,*cloud);





/* Ideia
* filtrar pontos acima do chão (hardcode < 1.4 m)
* tirar media
* gerar posição no chão com a media

*/
// TODO Ajeitar hardcoded floor


//Processar nuvem
	pcl::PassThrough<pcl::PointXYZ> pfilter;
  pcl::PointXYZ min,max;
	pfilter.setInputCloud(cloud);
	pfilter.setFilterFieldName("x");
	pfilter.setFilterLimits(0,lidar_height-0.1); //hardcoded floor
	pfilter.filter(*cloud_filtered);


//TODO ideia -> ICP 2D entre frames, descobre a translacao e filtra

  int n = cloud_filtered->size();
  float x_avg = 0;
  float y_avg = 0;
  for(int i=0;i<n;++i){
    y_avg += cloud_filtered->points[i].y;

  }

  y_avg /= n;
  
  geometry_msgs::PointStamped position_in;
  position_in.point.x = lidar_height; //hardcoded floor
  position_in.point.y = y_avg;
  position_in.point.z = 0;

  geometry_msgs::PointStamped position_out;
  
  geometry_msgs::TransformStamped tf_point = tf_buffer->lookupTransform("map",pc->header.frame_id,ros::Time(0));
  tf2::doTransform(position_in,position_out,tf_point);


  //ROS_INFO("in %f %f %f",position_in.point.x,position_in.point.y,position_in.point.z);
  //ROS_INFO("ou %f %f %f",position_out.point.x,position_out.point.y,position_out.point.z);




float x = y_avg;
ROS_INFO("x = %f",x);

std_msgs::UInt8 msg;
if(x < x_max && x > x_min){ // AQUI DEFINIMOS A FAIXA. PODE SER UM MIX COM SENSOR TOP TBM! PARAMETRIZAR 
  start_merge = true;
  msg.data = 1;
} else {
  start_merge = false;
    msg.data = 0;
}

merge_signal.publish(msg);

// Geramos um frame na posição desse x
static tf2_ros::TransformBroadcaster br;
geometry_msgs::TransformStamped transformStamped;

transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = "map";
//TODO parametrizar obj
transformStamped.child_frame_id = profiler_frame;

tf2::Quaternion q;
q.setRPY(0,0,0);
geometry_msgs::Quaternion q_msg = tf2::toMsg(q);

transformStamped.transform.rotation = q_msg;

transformStamped.transform.translation.x = position_out.point.x;
transformStamped.transform.translation.y = position_out.point.y;
transformStamped.transform.translation.z = 0;

br.sendTransform(transformStamped);
//ROS_INFO("Transform");


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bulk_scan_profiler");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber front_laser = nh.subscribe("/cloud_profiler",10,lidar_callback);
  // TODO Transformar em service
  merge_signal = nh.advertise<std_msgs::UInt8>("merge_signal",5);
  
  


  if (! private_nh.getParam("x_min",x_min) ) {
    ROS_ERROR("sete o parametro 'x_min'!");
    return -1;
  }

  if (! private_nh.getParam("x_max",x_max) ) {
    ROS_ERROR("sete o parametro 'x_max'!");
    return -1;

  }

  if (! private_nh.getParam("profiler_frame",profiler_frame) ) {
    ROS_WARN("'profiler_frame' not set. Using default -> %s",profiler_frame.c_str());
  }

  if( ! private_nh.getParam("lidar_height",lidar_height)) {
    ROS_ERROR("sete o parametro 'lidar_height' !");
    return -1;
  }
  
  tf_buffer = new tf2_ros::Buffer;
	tf_listener = new tf2_ros::TransformListener(*tf_buffer);

  

  ros::spin();


  delete tf_buffer;
  delete tf_listener;
}


