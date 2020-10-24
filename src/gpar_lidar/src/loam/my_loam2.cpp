/* Author : Marcus Forte <davi2812@dee.ufc.br> */

// This LOAM align and forms a map at a fixed rate

#include <ros/ros.h> //General Ros stuff

// PCL
#include <pcl/point_types.h>				 // PCL types .. i.e PointXYZ
#include <pcl_conversions/pcl_conversions.h> //PCL to ROS | ROS to PCL
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/distances.h>

// ROS Messages
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>		 //receive messages to manipulate cloud
#include <sensor_msgs/PointCloud2.h> // Process PC2 message
#include "geometry_msgs/TransformStamped.h"

// TF2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.h" // Allows direct transform application on ros message4

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT; // Define a templated type of pointcloud

// Map Publishing
PointCloudT::Ptr map_cloud;
ros::Publisher map_cloud_publisher;

// Map Parameters
float map_voxel_res = 0.01;

//Global transform
Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();

//Odometry variable
PointCloudT::Ptr current_cloud;

void timerCallback(const ros::TimerEvent &event)
{
	static bool first_cloud = true;

	if (first_cloud && (current_cloud->size() > 0))
	{
		first_cloud = false;
		*map_cloud += *current_cloud;
	}
	else if (first_cloud == false)
	{
		PointCloudT::Ptr aligned_cloud = boost::make_shared<PointCloudT>();
		static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		static pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr tf_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
		icp.setMaxCorrespondenceDistance(0.2);
		icp.setMaximumIterations(50);
		icp.setTransformationEpsilon(1e-8);
		icp.setTransformationEstimation(tf_2d);

		icp.setInputSource(current_cloud);
		icp.setInputTarget(map_cloud);
		icp.align(*aligned_cloud, global_transform);

		Eigen::Matrix4f correction_transform = icp.getFinalTransformation();
		std::cout << "Correction -> " << correction_transform << std::endl;
		//Smoothing required. KF maybe
		global_transform = correction_transform;

		*map_cloud += *aligned_cloud;
		pcl::VoxelGrid<pcl::PointXYZ> voxel;
		voxel.setInputCloud(map_cloud);
		voxel.setLeafSize(map_voxel_res, map_voxel_res, map_voxel_res); //preserve voxel
		voxel.filter(*map_cloud);
	}

	sensor_msgs::PointCloud2::Ptr out_msg = boost::make_shared<sensor_msgs::PointCloud2>();
	pcl::toROSMsg(*map_cloud, *out_msg);
	out_msg->header.frame_id = "map";
	map_cloud_publisher.publish(out_msg);
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
{
	PointCloudT::Ptr input_cloud = boost::make_shared<PointCloudT>();
	pcl::fromROSMsg<pcl::PointXYZ>(*pc_msg, *input_cloud);

	static pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(input_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0.2, 8);
	pass.filter(*input_cloud);

	*current_cloud = *input_cloud;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_loam2");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	std::string cloud_topic = nh.resolveName("cloud");

	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 100, cloud_callback);
	map_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 100);

	// Map
	ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);
	map_cloud = boost::make_shared<PointCloudT>();
	// tf2_ros::Buffer tfBuffer;
	// tf2_ros::TransformListener tfListener(tfBuffer);
	static bool first_cloud = true;

	//Odometry variables
	current_cloud = boost::make_shared<PointCloudT>();
	PointCloudT::Ptr previous_cloud = boost::make_shared<PointCloudT>();
	PointCloudT::Ptr aligned = boost::make_shared<PointCloudT>(); //dummy
	Eigen::Matrix4f current_transform;
	ros::Rate rate(20);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr tf_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
	icp.setMaxCorrespondenceDistance(0.2);
	icp.setMaximumIterations(50);
	icp.setTransformationEpsilon(1e-8);
	icp.setTransformationEstimation(tf_2d);

	// Odometry happens here
	while (ros::ok())
	{
		ros::spinOnce(); // process callbacks

		//Odometry
		if (previous_cloud->size() == 0)
		{
			*previous_cloud = *current_cloud;
			continue;
		}

		if (previous_cloud->header.seq == current_cloud->header.seq)
		{
			continue;
		}

		icp.setInputSource(current_cloud);
		icp.setInputTarget(previous_cloud);
		icp.align(*aligned);

		current_transform = icp.getFinalTransformation();

		global_transform = global_transform * current_transform;

		Eigen::Matrix3f rotation = global_transform.block<3, 3>(0, 0);
		Eigen::Vector3f rot_vec = rotation.eulerAngles(0, 1, 2);
		Eigen::Quaternionf rot_quat(rotation);

		//Update transform
		static tf2_ros::TransformBroadcaster br; //tf Broadcaster
		geometry_msgs::TransformStamped tf_transform;
		// This is generating ros warnings ..
		tf_transform.header.stamp = pcl_conversions::fromPCL(current_cloud->header.stamp);
		// TODO Make parametrized
		tf_transform.header.frame_id = "map";
		tf_transform.child_frame_id = "cloud";
		tf_transform.transform.translation.x = global_transform(0, 3);
		tf_transform.transform.translation.y = global_transform(1, 3);
		tf_transform.transform.translation.z = 0;
		tf_transform.transform.rotation.w = rot_quat.w();
		tf_transform.transform.rotation.x = rot_quat.x();
		tf_transform.transform.rotation.y = rot_quat.y();
		tf_transform.transform.rotation.z = rot_quat.z();
		br.sendTransform(tf_transform);

		*previous_cloud = *current_cloud;
		rate.sleep();
	}
}
