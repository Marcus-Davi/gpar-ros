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


#include "sensor_msgs/PointCloud2.h"
#include <tf2_ros/transform_broadcaster.h>

#include <mutex>
#include "Features.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

// static PointCloudT::Ptr cloud_map;


static PointCloudT::Ptr last_scan;

PointCloudT::Ptr aligned(new PointCloudT);

// ros::Publisher map_publisher;
ros::Publisher processed_publisher;

std::mutex g_lock;

Eigen::Matrix4f current_transform;
Eigen::Matrix4f odometry_transform = Eigen::Matrix4f::Identity();

static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //
// static pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //


ros::Time current_time;
// Mappig
void timerCallback(const ros::TimerEvent &ev)
{
}

//Laser Scan
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

	static PointCloudT::Ptr cloud_in(new PointCloudT);
	static PointCloudT::Ptr current_scan_raw(new PointCloudT);
	static PointCloudT::Ptr current_scan_DS(new PointCloudT);

	pcl::fromROSMsg(*msg, *cloud_in);

	auto start = std::chrono::high_resolution_clock::now();
	pcl::PassThrough<pcl::PointXYZ> pass_through;
	pass_through.setInputCloud(cloud_in);
	pass_through.setFilterFieldName("x");
	pass_through.setFilterLimits(0, 0.05); // remove weird points close to origin
	pass_through.setNegative(true);
	pass_through.filter(*current_scan_raw);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(current_scan_raw);
	sor.setMeanK(20);
	sor.setStddevMulThresh(0.2);
	// sor.filter(*current_scan_DS);

	pcl::VoxelGrid<pcl::PointXYZ> voxel;
	voxel.setInputCloud(current_scan_raw);
	voxel.setLeafSize(0.05,0.05,0.05);
	voxel.filter(*current_scan_DS);

	auto elapsed = std::chrono::high_resolution_clock::now() - start;
	ROS_WARN("Filter time: %ld us", std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());


	//first
	if (last_scan->points.size() == 0 || current_scan_DS->header.stamp < last_scan->header.stamp)
	{
		*last_scan = *current_scan_raw;
		return;
	}

	// process odometry
	start = std::chrono::high_resolution_clock::now();
	icp.setInputCloud(current_scan_DS);
	icp.setInputTarget(last_scan);

	// static pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr te_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
	// icp.setTransformationEstimation(te_2d);
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setMaximumIterations(150);
	icp.setTransformationEpsilon(1e-7);
	
	icp.align(*aligned);
	elapsed = std::chrono::high_resolution_clock::now() - start;
	ROS_WARN("Registration time: %ld us", std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
	ROS_WARN("Registration score: %f",icp.getFitnessScore());
	

	current_transform = icp.getFinalTransformation();
	odometry_transform = odometry_transform * current_transform;

	current_time = pcl_conversions::fromPCL(current_scan_raw->header.stamp);
	*last_scan = *current_scan_DS; // update 

	sensor_msgs::PointCloud2 processed_msg;
	pcl::toROSMsg(*current_scan_DS,processed_msg);
	
	processed_msg.header.frame_id = "cloud";
	processed_msg.header.stamp = msg->header.stamp;
	processed_publisher.publish(processed_msg);



	// Features::EdgeDetection(cloud_in,cloud_features,6,12);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "mloam");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	// cloud_map.reset(new PointCloudT);
	
	last_scan.reset(new PointCloudT);

	ros::Subscriber lidar_sub = nh.subscribe("cloud", 10, cloudCallback);
	processed_publisher = nh.advertise<sensor_msgs::PointCloud2>("processed_cloud",2);
	// map_publisher = nh.advertise<sensor_msgs::PointCloud2>("cloud_map", 10);



	tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped odometry_tf;

	PointCloudT::Ptr features(new PointCloudT);
	PointCloudT::Ptr edges(new PointCloudT);
	PointCloudT::Ptr planar(new PointCloudT);

	ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ROS_INFO("Waiting scan...");
	while (last_scan->size() == 0)
	{
		ros::spinOnce(); // Wait scan...;
	}

	ros::Rate r(10);
	// ROS_INFO("starting ")
	

	ros::Time last_time;
	while (ros::ok())
	{
		// Check correctness...

		Eigen::Matrix3f R = odometry_transform.block<3, 3>(0, 0);
		Eigen::Quaternionf rot_q(R);


		
		odometry_tf.header.frame_id = "map";
		odometry_tf.header.stamp = current_time;
		odometry_tf.child_frame_id = "cloud";

		

		odometry_tf.transform.translation.x = odometry_transform(0, 3);
		odometry_tf.transform.translation.y = odometry_transform(1, 3);
		odometry_tf.transform.translation.z = 0;

		odometry_tf.transform.rotation.w = rot_q.w();
		odometry_tf.transform.rotation.x = rot_q.x();
		odometry_tf.transform.rotation.y = rot_q.y();
		odometry_tf.transform.rotation.z = rot_q.z();


		//avoid warnings
		if ( odometry_tf.header.stamp > last_time){
		br.sendTransform(odometry_tf);
		last_time = odometry_tf.header.stamp;
		}

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}