/* Author : Marcus Forte <davi2812@dee.ufc.br> */

#include <ros/ros.h>

// PCL
#include <pcl/point_types.h>				 // PCL types .. i.e PointXYZ
#include <pcl_conversions/pcl_conversions.h> //PCL to ROS | ROS to PCL
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>

// PCL ROS
#include <pcl_ros/filters/passthrough.h>

#include "sensor_msgs/PointCloud2.h"
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
// #include "Features.h"

// Octopmap
#include <octomap/octomap.h>
#include "octomap_conversions.h"

// Comment here for pure cloud slam
// #define USE_OCTOMAP

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

ros::Publisher map_pub;
ros::Publisher currentFrame;

int init_samples;
int scan_samples;

int delay_samples = 10;

pcl::PCLPointCloud2 global_map;

void timerCallback(const ros::TimerEvent &event)
{

	sensor_msgs::PointCloud2 map_msg;

	pcl_conversions::fromPCL(global_map, map_msg);

	map_msg.header.frame_id = "map";
	map_msg.header.stamp = ros::Time::now();
	map_pub.publish(map_msg);
	ROS_INFO("Map published.. pts: %d", global_map.height * global_map.width);
}

// 10 Hz
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_in)
{
	static int init_count = 0;
	static int scan_count = 0;
	static int delay_count = 0;
	static PointCloudT::Ptr cloud_pcl(new PointCloudT);
	cloud_pcl->header.frame_id = "livox_frame";
	cloud_pcl->header.stamp = pcl_conversions::toPCL(cloud_in->header.stamp);
	ROS_WARN_ONCE("Got Lidar");

	if(delay_count < delay_samples){
		delay_count++;
		return;
	}
	if (init_count < init_samples)
	{
		pcl::PCLPointCloud2 input;
		pcl_conversions::toPCL(*cloud_in, input);

		global_map += input;
		init_count++;
	}

	if (scan_count == scan_samples)
	{

		scan_count = 0;

		currentFrame.publish(cloud_pcl);
		cloud_pcl->clear();

	} else {
		PointCloudT input_pcl;
		pcl::fromROSMsg(*cloud_in, input_pcl);
		*cloud_pcl += input_pcl;
		scan_count++;
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mLoam3");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh_private.param("init_samples", init_samples, 50);
	nh_private.param("scan_samples", scan_samples, 10);

	std::string cloud_topic = nh.resolveName("cloud");
	ros::Subscriber lidar_sub = nh.subscribe(cloud_topic, 100, lidarCallback);
	map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 10);
	currentFrame = nh.advertise<PointCloudT>("currentFrame", 10);

	ros::Timer timer = nh.createTimer(ros::Duration(1), timerCallback);

	ros::spin();

	return 0;
}