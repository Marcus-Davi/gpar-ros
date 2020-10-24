/* Author : Marcus Forte <davi2812@dee.ufc.br> */

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

Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();

geometry_msgs::Pose2D update_pose;

// Debug Vis
static pcl::visualization::PCLVisualizer::Ptr viewer;
int vp0, vp1;

bool first_cloud = true;

// Parameters
float icp_voxel_res = 0.5;

PointCloudT::Ptr current_cloud = boost::make_shared<PointCloudT>();
PointCloudT::Ptr previous_cloud = boost::make_shared<PointCloudT>();

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
{

	PointCloudT::Ptr input_cloud = boost::make_shared<PointCloudT>();
	pcl::fromROSMsg<pcl::PointXYZ>(*pc_msg, *input_cloud);

	static pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(input_cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0.2, 8);
	pass.filter(*input_cloud);

	pcl::VoxelGrid<pcl::PointXYZ> voxel;
	voxel.setInputCloud(input_cloud);
	// voxel.setLeafSize(icp_voxel_res, icp_voxel_res, icp_voxel_res);
	// voxel.filter(*current_cloud);
	*current_cloud = *input_cloud;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "slam_icp");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	std::string cloud_topic = nh.resolveName("cloud");

	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 100, cloud_callback);

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr tf_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setMaximumIterations(100);
	icp.setTransformationEpsilon(1e-8);
	icp.setTransformationEstimation(tf_2d);

	PointCloudT::Ptr aligned(new PointCloudT);
	Eigen::Matrix4f current_transform;

	ros::Rate rate(10); //Odometry

	while (ros::ok())
	{
		ros::spinOnce(); // process callbacks

		//Odometry
		if (previous_cloud->size() == 0)
		{
			*previous_cloud = *current_cloud;
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
		tf_transform.header.stamp = pcl_conversions::fromPCL(current_cloud->header.stamp);
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
