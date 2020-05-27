#include <ros/ros.h> //General Ros stuff
#include <sensor_msgs/PointCloud2.h> // Process PC2 message
#include <pcl/point_types.h> // PCL types .. i.e PointXYZ
#include <pcl_conversions/pcl_conversions.h> //PCL to ROS | ROS to PCL
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h" // Allows direct transform application on ros message

#include "std_msgs/String.h" //receive messages to manipulate cloud 

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT; // Define a templated type of pointcloud


std::string world_frame;
ros::Publisher pub_world;
ros::Publisher pub_merged;
tf2_ros::TransformListener *tf_listener;
tf2_ros::Buffer *tf_buffer;

PointCloudT::Ptr cloud_merged = boost::make_shared<PointCloudT>();

void msg_callback(const std_msgs::String::ConstPtr& str_msg){
		ROS_INFO("got msg");
if(str_msg->data == "reset")
{
	ROS_WARN("reset cloud");
	cloud_merged->clear();
}
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pc){
static PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>(); //Make pointer to input cloud
// pcl::fromROSMsg(*pc,*cloud);
// ROS_INFO("callback called!");
// PointCloudT::Ptr world_cloud = boost::make_shared<PointCloudT>();
// ROS_INFO("source frame = %s",cloud->header.frame_id.c_str());
// ROS_INFO("world frame = %s",world_frame.c_str());

sensor_msgs::PointCloud2::Ptr pc_transformed = boost::make_shared<sensor_msgs::PointCloud2>();
// It is a good practive to verify availability of transform using tf_buffer methods : catch/except
try { 

		geometry_msgs::TransformStamped tf = tf_buffer->lookupTransform(world_frame,pc->header.frame_id,ros::Time(0));
///pcl_ros::transformPointCloud(world_frame,*cloud,*world_cloud,*tf_buffer); // Transforms point clouds. 
tf2::doTransform(*pc,*pc_transformed,tf);
// good practice to verify transform sucess

pcl::fromROSMsg(*pc_transformed,*cloud);
// maybe there is a way to directly merge point clouds msgs
*cloud_merged += *cloud;
sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();

pcl::toROSMsg(*cloud_merged,*msg);
msg->header.stamp = pc->header.stamp;
msg->header.frame_id = world_frame;
// pcl::toROSMsg(*world_cloud,*msg);
// msg->header.stamp = pc->header.stamp;
// msg->header.frame_id = world_frame;
pub_world.publish(pc_transformed);
pub_merged.publish(msg);

} catch(tf2::TransformException &ex) {
ROS_WARN("%s", ex.what());
}
}




int main(int argc,char** argv){
	ros::init(argc, argv, "simple_cloud_combinator");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
std::string input_cloud_node;


 if (!nh_private.getParam("fixed_frame", world_frame))
 {
  world_frame = "map";
  ROS_WARN("Need to set parameter '_fixed_frame'.. set to \"%s\"",world_frame
.c_str());
 }
	
	if(!nh_private.getParam("input_cloud",input_cloud_node)){
		ROS_FATAL("please set 'input cloud parameter' !");
		return -1;
	}
	ROS_INFO("input cloud -> %s",input_cloud_node.c_str());
	ros::Subscriber cloud_sub = nh.subscribe(input_cloud_node,100,cloud_callback);

	ros::Subscriber cloud_msgs = nh_private.subscribe("msgs",100,msg_callback);

	tf_buffer = new tf2_ros::Buffer;
	tf_listener = new tf2_ros::TransformListener(*tf_buffer);

	ROS_INFO("running..");

	pub_world = nh.advertise<sensor_msgs::PointCloud2>("world_cloud",10);
	pub_merged = nh.advertise<sensor_msgs::PointCloud2>("merged_cloud",10);
	ros::spin();

}
