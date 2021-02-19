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

#include "Features.h"

#include "sensor_msgs/PointCloud2.h"

#include <tf2_ros/transform_broadcaster.h>

#include <mutex>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

static PointCloudT::Ptr cloud_map;

static PointCloudT::Ptr current_scan;
// static PointCloudT::Ptr last_scan;

PointCloudT::Ptr aligned(new PointCloudT);

ros::Publisher map_publisher;

std::mutex g_lock;

Eigen::Matrix4f current_transform;
Eigen::Matrix4f odometry_transform = Eigen::Matrix4f::Identity();

static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //
// static pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //

// Mappig
void timerCallback(const ros::TimerEvent &ev)
{

	*cloud_map += *aligned; // Registration

	pcl::VoxelGrid<pcl::PointXYZ> downsample;
	downsample.setInputCloud(cloud_map);
	downsample.setLeafSize(0.01, 0.01, 0.01);
	downsample.filter(*cloud_map);

	sensor_msgs::PointCloud2 map_msg;
	pcl::toROSMsg(*cloud_map, map_msg);
	map_msg.header.stamp = pcl_conversions::fromPCL(current_scan->header.stamp);
	map_msg.header.frame_id = "map";
	map_publisher.publish(map_msg);
}

//Laser Scan
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

	static PointCloudT::Ptr cloud_in(new PointCloudT);

	pcl::fromROSMsg(*msg, *cloud_in);

	if (cloud_map->size() == 0)
	{
		*cloud_map += *cloud_in;
		ROS_INFO("Map created");
		return;
	}

	pcl::PassThrough<pcl::PointXYZ> pass_through;
	pass_through.setInputCloud(cloud_in);
	pass_through.setFilterFieldName("x");
	pass_through.setFilterLimits(0, 0.05); // remove weird points close to origin
	pass_through.setNegative(true);
	pass_through.filter(*cloud_in);

	*current_scan = *cloud_in;

	// Features::EdgeDetection(cloud_in,cloud_features,6,12);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "mloam");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	cloud_map.reset(new PointCloudT);
	current_scan.reset(new PointCloudT);

	ros::Subscriber lidar_sub = nh.subscribe("cloud", 10, cloudCallback);
	map_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_map", 10);

	// static pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr te_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
	// icp.setTransformationEstimation(te_2d);
	icp.setMaxCorrespondenceDistance(0.3);
	icp.setMaximumIterations(200);
	icp.setTransformationEpsilon(1e-8);

	tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped odometry_tf;

	PointCloudT::Ptr features(new PointCloudT);
	PointCloudT::Ptr edges(new PointCloudT);
	PointCloudT::Ptr planar(new PointCloudT);

	ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);

	// ros::AsyncSpinner spinner(2);
	// spinner.start();

	ROS_INFO("Waiting scan...");
	while (current_scan->size() == 0)
	{
		ros::spinOnce(); // Wait scan...;
	}

	ros::Rate r(10);

	// Main Loop @ 10HZ
	while (ros::ok())
	{

		icp.setInputSource(current_scan);
		icp.setInputTarget(cloud_map); // align with map
		icp.align(*aligned, odometry_transform);

		// Check correctness...

		odometry_transform = icp.getFinalTransformation(); //between scans

		Eigen::Matrix3f R = odometry_transform.block<3, 3>(0, 0);
		Eigen::Quaternionf rot_q(R);

		odometry_tf.header.frame_id = "map";
		odometry_tf.header.stamp = pcl_conversions::fromPCL(current_scan->header.stamp);
		odometry_tf.child_frame_id = "cloud";

		odometry_tf.transform.translation.x = odometry_transform(0, 3);
		odometry_tf.transform.translation.y = odometry_transform(1, 3);
		odometry_tf.transform.translation.z = 0;

		odometry_tf.transform.rotation.w = rot_q.w();
		odometry_tf.transform.rotation.x = rot_q.x();
		odometry_tf.transform.rotation.y = rot_q.y();
		odometry_tf.transform.rotation.z = rot_q.z();

		br.sendTransform(odometry_tf);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}