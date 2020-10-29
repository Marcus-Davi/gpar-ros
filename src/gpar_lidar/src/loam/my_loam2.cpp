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
#include <pcl/search/kdtree.h>
#include <pcl/common/distances.h>
#include <pcl/filters/statistical_outlier_removal.h>

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

#include <octomap/octomap.h>

#include "octomap_conversions.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT; // Define a templated type of pointcloud

// Map Publishing

ros::Publisher map_cloud_publisher;

// Map Parameters
double input_voxel_res;
octomap::OcTree *octmaptree;
bool use_sor;

//Global transform
Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();

//Odometry variable
PointCloudT::Ptr current_cloud;
PointCloudT::Ptr map_cloud;

std::mutex g_lock;

void timerCallback(const ros::TimerEvent &event)
{
	static bool first_cloud = true;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
{
	PointCloudT::Ptr input_cloud = boost::make_shared<PointCloudT>();
	pcl::fromROSMsg<pcl::PointXYZ>(*pc_msg, *input_cloud);

	static pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(input_cloud);
	// pass.setFilterFieldName("x");
	// pass.setFilterLimits(-20, 20);
	// // pass.filter(*input_cloud);
	// pass.setFilterFieldName("y");
	// // pass.filter(*input_cloud);
	pass.setFilterLimits(-0.5, 0.5); // hehe
	pass.setFilterFieldName("z");
	pass.filter(*input_cloud);

	

	static pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	if (use_sor)
	{
		sor.setInputCloud(input_cloud);
		sor.setMeanK(50);
		sor.setStddevMulThresh(1.0);
		sor.filter(*input_cloud);
	}

	pcl::VoxelGrid<pcl::PointXYZ> voxel;
	voxel.setInputCloud(input_cloud);
	voxel.setLeafSize(input_voxel_res, input_voxel_res, input_voxel_res);
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

	//Get parameters
	double map_res;
	double prob_occ;
	double prob_free;
	double prob_thres;
	int max_icp_it;
	std::string icp_method;
	nh_private.getParam("map_resolution", map_res);
	nh_private.param("prob_occ", prob_occ, 0.9);
	nh_private.param("prob_free", prob_free, 0.4);
	nh_private.param("prob_thres", prob_thres, 0.8);
	nh_private.param("input_voxel_size", input_voxel_res, 0.1);
	nh_private.param("use_sor", use_sor, false);
	nh_private.param<std::string>("icp_method", icp_method, "icp");
	nh_private.param<int>("icp_iterations", max_icp_it, 10);

	octmaptree = new octomap::OcTree(map_res);
	octmaptree->setProbHit(prob_occ);
	octmaptree->setProbMiss(prob_free);
	octmaptree->setOccupancyThres(prob_thres);

	ROS_WARN("map resolution -> %f", map_res);
	ROS_WARN("prob_occ -> %f", prob_occ);
	ROS_WARN("prob_free -> %f", prob_free);
	ROS_WARN("prob_thres -> %f", prob_thres);
	ROS_WARN("input_voxel_size -> %f", input_voxel_res);
	static pcl::Registration<pcl::PointXYZ,pcl::PointXYZ>::Ptr icp;

	if(icp_method == "icp"){
		icp = boost::make_shared<pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>>();
		ROS_WARN("using : ICP");
	} else if(icp_method == "gicp") {
		ROS_WARN("using : GICP");
		icp = boost::make_shared<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ>>();
	} else {
		ROS_ERROR("invalid ICP method");
		ros::shutdown();
		exit(-1);
	}

	

	std::string cloud_topic = nh.resolveName("cloud");

	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 100, cloud_callback);
	map_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 100);

	static bool first_cloud = true;

	//Odometry variables
	current_cloud = boost::make_shared<PointCloudT>();
	map_cloud = boost::make_shared<PointCloudT>();
	PointCloudT::Ptr aligned = boost::make_shared<PointCloudT>(); //dummy
	
	// PointCloudT::Ptr previous_cloud = boost::make_shared<PointCloudT>();
	
	Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
	ros::Rate rate(5);

	// pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	// pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	// pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr tf_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
	// icp.setMaxCorrespondenceDistance(0.2);
	// icp.setMaximumIterations(1);
	// icp.setTransformationEpsilon(1e-6);
	// icp.setTransformationEstimation(tf_2d);
	// ros::AsyncSpinner spinner(1);
	// spinner.start();
	// Odometry happens here
	while (ros::ok())
	{
		ros::spinOnce();


		if (first_cloud && (current_cloud->size() > 0))
		{
			first_cloud = false;
			octomap::Pointcloud pc;
			octomap::point3d origin(0, 0, 0);
			g_lock.lock();
			pcl2octopc(*current_cloud, pc);
			g_lock.unlock();
			
			octmaptree->insertPointCloudRays(pc, origin);
			// octmaptree->insertPointCloudRays(pc, origin);
			// *map_cloud += *current_cloud;
			// g_lock.unlock();
			continue;
		}
		else if (first_cloud == false)
		{
			// Mapping Algorithm here
			PointCloudT::Ptr aligned_cloud = boost::make_shared<PointCloudT>();
			
			// static pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
			// static pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr tf_2d(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
			// pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZ,pcl::PointXYZ>::Ptr tf_3d(new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointXYZ,pcl::PointXYZ>);
			icp->setMaxCorrespondenceDistance(0.2);
			icp->setMaximumIterations(max_icp_it);
			icp->setTransformationEpsilon(1e-7);
			// icp->setTransformationEstimation(tf_2d);
			g_lock.lock();
			icp->setInputSource(current_cloud); //Scan cloud

			//Convert map_cloud to octomap

			octomap2pcl(*octmaptree, *map_cloud); //get occupied nodes

			icp->setInputTarget(map_cloud);

			// This is where we can use an estimated guess
			std::cout << "aligning..." << std::endl;
			icp->align(*aligned_cloud, global_transform);
			g_lock.unlock();

			global_transform = icp->getFinalTransformation();
			std::cout << "Correction -> " << global_transform << std::endl;
			// Guess is corrected. Use Smoothing
			// global_transform = correction_transform;

			//Filtering, Optimizations advised here

			// Update octomap
			boost::shared_ptr<octomap::Pointcloud> octomap_pc(new octomap::Pointcloud);
			pcl2octopc(*aligned_cloud, *octomap_pc);
			octomap::point3d origin(global_transform(3, 0), global_transform(3, 1), global_transform(3, 2));

			ROS_WARN("inserting ray..");
			octmaptree->insertPointCloudRays(*octomap_pc, origin, -1, false);
			// octmaptree->insertPointCloud(*octomap_pc,origin,10,false,true);
			ROS_WARN("done");
			ROS_WARN("writing binary...");
			octmaptree->writeBinary("MAP.bt");
			ROS_WARN("done");
		}

		sensor_msgs::PointCloud2::Ptr out_msg = boost::make_shared<sensor_msgs::PointCloud2>();
		pcl::toROSMsg(*map_cloud, *out_msg);
		out_msg->header.frame_id = "map";
		map_cloud_publisher.publish(out_msg);

		Eigen::Matrix3f rotation = global_transform.block<3, 3>(0, 0);
		// Eigen::Vector3f rot_vec = rotation.eulerAngles(0, 1, 2);
		Eigen::Quaternionf rot_quat(rotation);

		//Update transform
		static tf2_ros::TransformBroadcaster br; //tf Broadcaster
		geometry_msgs::TransformStamped tf_transform;
		// This is generating ros warnings ..
		tf_transform.header.stamp = ros::Time::now();
		// TODO Make parametrized
		tf_transform.header.frame_id = "map";
		tf_transform.child_frame_id = current_cloud->header.frame_id;

		tf_transform.transform.translation.x = global_transform(0, 3);
		tf_transform.transform.translation.y = global_transform(1, 3);
		tf_transform.transform.translation.z = 0;
		tf_transform.transform.rotation.w = rot_quat.w();
		tf_transform.transform.rotation.x = rot_quat.x();
		tf_transform.transform.rotation.y = rot_quat.y();
		tf_transform.transform.rotation.z = rot_quat.z();
		br.sendTransform(tf_transform);

		// g_lock.lock();
		// // *previous_cloud = *current_cloud;
		// g_lock.unlock();

		rate.sleep();
	}
}
