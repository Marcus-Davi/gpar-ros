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

PointCloudT::Ptr mapCloud(new PointCloudT);
PointCloudT::Ptr currentCloud(new PointCloudT);

Eigen::Matrix4f global_tf = Eigen::Matrix4f::Identity();
ros::Publisher mapPub;

// Resolution
std::shared_ptr<octomap::OcTree> octmap;

//ICP
float corr_dist;
float voxel_res;
static bool mapEmpty = true;


std::mutex g_lock;

void mappingCallback(const ros::TimerEvent &event)
{
	// check if we have a map
	if(mapEmpty){
		if (currentCloud->points.size() != 0)
		{
#ifdef USE_OCTOMAP
			octomap::Pointcloud octo_pc;
			pcl2octopc(*currentCloud, octo_pc);
			octomap::point3d origin(0, 0, 0);
			octmap->insertPointCloudRays(octo_pc, origin);

#else

			*mapCloud = *currentCloud;

#endif
		mapEmpty = false;
			return;
		} else { //not ready
		return;
		}
	}

	// next iterations
	ROS_WARN("Compute mapping");
	auto start = std::chrono::high_resolution_clock::now();
	// Use initieal guess of odometry to register to a map
	PointCloudT::Ptr currCloudTf(new PointCloudT);
	static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	// pcl::transformPointCloud(*currentCloud,currCloudTf,global_tf);
	// g_lock.lock();
	icp.setInputSource(currentCloud);

#ifdef USE_OCTOMAP
	// retrieve from octopmap
	octomap2pcl(*octmap, *mapCloud);
#endif

	// registration
	icp.setInputTarget(mapCloud);
	icp.setMaxCorrespondenceDistance(0.25);
	icp.setMaximumIterations(100);
	icp.setTransformationEpsilon(1e-8);
	static pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr tf_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
	icp.setTransformationEstimation(tf_2d);
	icp.align(*currCloudTf, global_tf);
	
	global_tf = icp.getFinalTransformation();

//octomap operations
#ifdef USE_OCTOMAP
	octomap::Pointcloud newScan;
	pcl2octopc(*currCloudTf, newScan);
	octomap::point3d origin(0, 0, 0); // scan already transfrmed
	octmap->insertPointCloudRays(newScan, origin);
	octomap2pcl(*octmap, *mapCloud);
#else
	*mapCloud += *currCloudTf;
	pcl::VoxelGrid<pcl::PointXYZ> voxel;
	voxel.setInputCloud(mapCloud);
	voxel.setLeafSize(voxel_res, voxel_res, voxel_res);
	voxel.filter(*mapCloud);
#endif

	auto end = std::chrono::high_resolution_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	ROS_INFO("Mapping ICP Time: %ld us | Fit Score: %f", elapsed, icp.getFitnessScore());
	// Publish map
	sensor_msgs::PointCloud2 mapMsg;
	pcl::toROSMsg(*mapCloud, mapMsg);
	mapMsg.header.frame_id = "map";
	mapMsg.header.stamp = ros::Time::now();	
	mapPub.publish(mapMsg);
	// g_lock.unlock();
}

void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	// g_lock.lock();
 	pcl::fromROSMsg(*msg, *currentCloud);
	pcl::PassThrough<pcl::PointXYZ> passthrough;
	passthrough.setInputCloud(currentCloud);
	passthrough.setFilterFieldName("x");
	passthrough.setFilterLimits(0.05, 10);
	passthrough.setFilterLimitsNegative(false);
	passthrough.filter(*currentCloud);
	// g_lock.unlock();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mloam");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	// Sampling
	float Ts;
	float Ts_map;

	nh_private.param<float>("odometry_time", Ts, 0.1);
	nh_private.param<float>("mapping_time", Ts_map, 0.5);

	nh_private.param<float>("corr_dist", corr_dist, 0.1);
	nh_private.param<float>("voxel_res", voxel_res, 0.1);

	//Octomap
	octmap = std::make_shared<octomap::OcTree>(voxel_res);
	octmap->setProbHit(0.92);
	octmap->setProbMiss(0.2);
	octmap->setOccupancyThres(0.5);
	octmap->setClampingThresMin(0.02);
	octmap->setClampingThresMax(0.98);
	octmap->setResolution(voxel_res);

	// ROS_WARN("corr dist = %f",corr_dist);

	ros::Subscriber sub = nh.subscribe("cloud", 1, scanCallback);

	ros::Rate r(1 / Ts);
	ros::Publisher laser_pub1 = nh.advertise<sensor_msgs::PointCloud2>("cloud1", 1);
	ros::Publisher laser_pub2 = nh.advertise<sensor_msgs::PointCloud2>("cloud2", 1);
	ros::Publisher laser_pub3 = nh.advertise<sensor_msgs::PointCloud2>("cloud3", 1);
	mapPub = nh.advertise<sensor_msgs::PointCloud2>("cloud_map", 1);
	sensor_msgs::PointCloud2 laser_msg1, laser_msg2, laser_msg3;

	PointCloudT::Ptr lastCloud(new PointCloudT);
	PointCloudT::Ptr tfCloud(new PointCloudT);

	tf2_ros::TransformBroadcaster br;

	ros::Timer mappingTimer = nh.createTimer(ros::Duration(Ts_map), mappingCallback);
	// ros::AsyncSpinner spinner(1);
	// spinner.start();
	

	// Odometry Loop
	while (ros::ok())
	{

		if (lastCloud->points.size() == 0)
		{ //empty last ?
			if (currentCloud->points.size() != 0)
			{ // new scan
				*lastCloud = *currentCloud;
			}
			else
			{
				ros::spinOnce();
				continue;
			}
		}

		// Filtering
		// g_lock.lock();
		auto start = std::chrono::high_resolution_clock::now();
		static pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(currentCloud);
		sor.setMeanK(20);
		sor.setStddevMulThresh(0.1);
		sor.filter(*currentCloud);

		static pcl::VoxelGrid<pcl::PointXYZ> voxel;
		voxel.setInputCloud(currentCloud);
		voxel.setLeafSize(voxel_res, voxel_res, voxel_res);
		voxel.filter(*currentCloud);
		auto end = std::chrono::high_resolution_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		ROS_INFO("Filtering time: %ld us", elapsed);

		// Compute Odometry between frames
		
		start = std::chrono::high_resolution_clock::now();
		static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(currentCloud);
		icp.setInputTarget(lastCloud);
		icp.setMaxCorrespondenceDistance(corr_dist);
		icp.setMaximumIterations(100);
		icp.setTransformationEpsilon(1e-8);
		static pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr tf_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
		icp.setTransformationEstimation(tf_2d);
		icp.align(*tfCloud);
		end = std::chrono::high_resolution_clock::now();
		elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
		ROS_INFO("ICP Time: %ld us | Fit Score: %f", elapsed, icp.getFitnessScore());
		
		

		Eigen::Matrix4f odom_tf = icp.getFinalTransformation();

		global_tf = global_tf * odom_tf;

		geometry_msgs::TransformStamped tf_stamped;
		tf_stamped.header.frame_id = "map";
		tf_stamped.child_frame_id = "cloud"; // laser base
		tf_stamped.header.stamp = ros::Time::now();
		tf_stamped.transform.translation.x = global_tf(0, 3); //x
		tf_stamped.transform.translation.y = global_tf(1, 3); //y
		tf_stamped.transform.translation.z = global_tf(2, 3); //z
		Eigen::Quaternionf q(global_tf.block<3, 3>(0, 0));
		tf_stamped.transform.rotation.w = q.w();
		tf_stamped.transform.rotation.x = q.x();
		tf_stamped.transform.rotation.y = q.y();
		tf_stamped.transform.rotation.z = q.z();

		br.sendTransform(tf_stamped);

		// pcl::toROSMsg(*currentCloud, laser_msg1);
		// pcl::toROSMsg(*lastCloud, laser_msg2);
		// pcl::toROSMsg(*tfCloud, laser_msg3);

		// laser_msg1.header.frame_id = "cloud";
		// laser_msg1.header.stamp = ros::Time::now();
		// laser_msg2.header.frame_id = "cloud";
		// laser_msg2.header.stamp = ros::Time::now();
		// laser_msg3.header.frame_id = "cloud";
		// laser_msg3.header.stamp = ros::Time::now();

		// laser_pub1.publish(laser_msg1);
		// laser_pub2.publish(laser_msg2);
		// laser_pub3.publish(laser_msg3);

		*lastCloud = *currentCloud;

		ros::spinOnce();
		r.sleep();
		// g_lock.unlock();
	}
}