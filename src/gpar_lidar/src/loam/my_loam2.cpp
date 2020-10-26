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

#include <mutex>

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

std::mutex g_lock;

void timerCallback(const ros::TimerEvent &event)
{
	static bool first_cloud = true;

	if (first_cloud && (current_cloud->size() > 0))
	{
		first_cloud = false;
		g_lock.lock();
		*map_cloud += *current_cloud;
		g_lock.unlock();
	}
	else if (first_cloud == false)
	{
		// Mapping Algorithm here
		PointCloudT::Ptr aligned_cloud = boost::make_shared<PointCloudT>();
		// static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		static pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		// static pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr tf_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
		// pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZ,pcl::PointXYZ>::Ptr tf_3d(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZ,pcl::PointXYZ>);
		icp.setMaxCorrespondenceDistance(0.3);
		icp.setMaximumIterations(50);
		icp.setTransformationEpsilon(1e-7);
		// icp.setTransformationEstimation(tf_2d);

		icp.setInputSource(current_cloud);
		icp.setInputTarget(map_cloud);

		// This is where we can use an estimated guess
		std::cout << "aligning..." << std::endl;
		icp.align(*aligned_cloud, global_transform);

		Eigen::Matrix4f correction_transform = icp.getFinalTransformation();
		std::cout << "Correction -> " << correction_transform << std::endl;
		// Guess is corrected. Use Smoothing
		global_transform = correction_transform;


		//Filtering, Optimizations advised here.
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
	// pass.setFilterFieldName("x");
	// pass.setFilterLimits(0.2, 0.8);
	// pass.filter(*input_cloud);

	pcl::VoxelGrid<pcl::PointXYZ> voxel;
	voxel.setInputCloud(input_cloud);
	voxel.setLeafSize(map_voxel_res,map_voxel_res,map_voxel_res);
	voxel.filter(*input_cloud);

	g_lock.lock();
	*current_cloud = *input_cloud;
	g_lock.unlock();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_loam2");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh_private.getParam("map_resolution",map_voxel_res);

	ROS_WARN("map resolution -> %f",map_voxel_res);

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
	// pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	// pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr tf_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
	icp.setMaxCorrespondenceDistance(0.2);
	icp.setMaximumIterations(2);
	icp.setTransformationEpsilon(1e-6);
	// icp.setTransformationEstimation(tf_2d);
	ros::AsyncSpinner spinner(1);
	spinner.start();
	// Odometry happens here
	while (ros::ok())
	{
		// ros::spinOnce(); // process callbacks

		//Odometry
		if (previous_cloud->size() == 0)
		{
			g_lock.lock();
			*previous_cloud = *current_cloud;
			g_lock.unlock();
			continue;
		}

		if (previous_cloud->header.seq == current_cloud->header.seq)
		{
			continue;
		}

		icp.setInputSource(current_cloud);
		icp.setInputTarget(previous_cloud);
		ROS_INFO("Odometyr ICP...");
		icp.align(*aligned);
		ROS_INFO("Odometyr OK!");
		
		current_transform = icp.getFinalTransformation();

		global_transform = global_transform * current_transform;

		Eigen::Matrix3f rotation = global_transform.block<3, 3>(0, 0);
		Eigen::Vector3f rot_vec = rotation.eulerAngles(0, 1, 2);
		Eigen::Quaternionf rot_quat(rotation);

		//Update transform
		static tf2_ros::TransformBroadcaster br; //tf Broadcaster
		geometry_msgs::TransformStamped tf_transform;
		// This is generating ros warnings ..
		tf_transform.header.stamp = ros::Time::now();
		// TODO Make parametrized
		tf_transform.header.frame_id = "map";
		tf_transform.child_frame_id = current_cloud->header.frame_id;;
		tf_transform.transform.translation.x = global_transform(0, 3);
		tf_transform.transform.translation.y = global_transform(1, 3);
		tf_transform.transform.translation.z = 0;
		tf_transform.transform.rotation.w = rot_quat.w();
		tf_transform.transform.rotation.x = rot_quat.x();
		tf_transform.transform.rotation.y = rot_quat.y();
		tf_transform.transform.rotation.z = rot_quat.z();
		br.sendTransform(tf_transform);

		g_lock.lock();
		*previous_cloud = *current_cloud;
		g_lock.unlock();
		
		rate.sleep();
	}
}
