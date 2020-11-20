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
#include "gridslam.h"
#include "occmap.h"

//Use octopmap here
// #define USE_OCTOMAP

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT; // Define a templated type of pointcloud

// Map Publishing

ros::Publisher map_cloud_publisher;

// Map Parameters
double input_voxel_res;
#ifdef USE_OCTOMAP
octomap::OcTree *octmaptree;
#else
myslam::OccMap *occmap;
#endif
double g_map_res;

//Global transform
Eigen::Matrix4d global_transform = Eigen::Matrix4d::Identity();

Eigen::Vector3d last_up_pose = Eigen::Vector3d::Zero();
Eigen::Vector3d global_transform_2d = Eigen::Vector3d::Zero();

//Odometry variable
PointCloudT::Ptr current_cloud = boost::make_shared<PointCloudT>();
PointCloudT::Ptr map_cloud;

std::mutex g_lock;

bool first_map = true;

void timerCallback(const ros::TimerEvent &event)
{
	static bool first_cloud = true;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
{
	PointCloudT::Ptr input_cloud = boost::make_shared<PointCloudT>();

	pcl::fromROSMsg<pcl::PointXYZ>(*pc_msg, *input_cloud);
	// ROS_INFO("LASER");
	if (first_map)
	{
		octomap::Pointcloud first_pc;
		octomap::point3d origin(0, 0, 0);
		g_lock.lock();
		pcl2octopc(*input_cloud, first_pc);
		g_lock.unlock();

#ifdef USE_OCTOMAP
octmaptree->insertPointCloudRays(first_pc, origin);// REPLACE
octomap2pcl(*octmaptree, *map_cloud); //get occupied nodes // REPLACE
#else
		occmap->insertPointCloudRays(first_pc, origin);
		occmap2pcl(*occmap,*map_cloud);
#endif

		

		sensor_msgs::PointCloud2::Ptr out_msg = boost::make_shared<sensor_msgs::PointCloud2>();
		pcl::toROSMsg(*map_cloud, *out_msg);
		out_msg->header.frame_id = "map";
		map_cloud_publisher.publish(out_msg);
		first_map = false;
	}
	else
	{
		Eigen::Quaterniond q;
		// pcl::PassThrough<pcl::PointXYZ> remover;
		// remover.setInputCloud(input_cloud);
		// remover.setFilterFieldName("x");
		// remover.setFilterLimits(-0.01, 0.01); //clear zeros
		// remover.setNegative(true);
		// remover.filter(*input_cloud);

		// g_lock.lock();
		// pcl::copyPointCloud(*input_cloud, *current_cloud);
		// g_lock.unlock();

		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_tf = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

		Eigen::Matrix<double, 1, 2> dm;
		Eigen::Matrix<double, 2, 3> jac;
		Eigen::Matrix3d Hessian = Eigen::Matrix3d::Identity(3, 3);
		Eigen::Vector3d dtr = Eigen::Vector3d::Zero();

		double funval;
		int maxIt = 5;

		auto start = std::chrono::high_resolution_clock::now();
		for (int j = 0; j < maxIt; ++j)
		{

			pcl::transformPointCloud(*input_cloud, *input_cloud_tf, global_transform);
			std::cout << "pts -> " << input_cloud_tf->size() << std::endl;

			for (int i = 0; i < input_cloud->size(); ++i)
			{
				pcl::PointXYZ endpoint = input_cloud_tf->points[i];

				// myslam::TransformEndpoint(endpoint,global_transform_2d);

#ifdef USE_OCTOMAP
				funval = 1 - myslam::mapAccess(*octmaptree, endpoint);			  // REPLACE
				dm = myslam::mapGradient<1, 2>(*octmaptree, endpoint);			  // -12 // REPLACE
				jac = myslam::modelGradient<2, 3>(endpoint, global_transform_2d); //REPLACE
#else
				funval = 1 - myslam::mapAccess(*occmap, endpoint);
				dm = myslam::mapGradient<1, 2>(*occmap, endpoint);
				jac = myslam::modelGradient<2, 3>(endpoint, global_transform_2d);
#endif
				// std::cout << "funval = " << funval << std::endl;
				// std::cout << "dm = " << dm << std::endl;
				// std::cout << "jac = " << jac << std::endl;
				dtr = dtr + (dm * jac).transpose() * funval;
				Hessian = Hessian + (dm * jac).transpose() * (dm * jac);
				// std::cout << "dtr = " << dtr << std::endl;
				// std::cout << "Hessian" << Hessian << std::endl;
			}
			Eigen::Vector3d searchdir = Eigen::Vector3d::Zero();
			if (Hessian(0, 0) != 0 && Hessian(1, 1) != 0)
			{
				searchdir = Hessian.inverse() * dtr;
				std::cout << "Search dir = " << searchdir << std::endl;
				std::cout << "Hessian dir = " << Hessian << std::endl;
			}

			global_transform_2d = global_transform_2d + searchdir; // Update

			global_transform(0, 3) = global_transform_2d[0];
			global_transform(1, 3) = global_transform_2d[1];
			global_transform(2, 3) = 0;

			q = Eigen::AngleAxisd(global_transform_2d[2], Eigen::Vector3d::UnitZ()); //
			Eigen::Matrix3d m = q.normalized().toRotationMatrix();
			global_transform.block<3, 3>(0, 0) = m;
		}
		auto end = std::chrono::high_resolution_clock::now();

		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		std::cout << "Time : " << duration.count() << " ms" << std::endl;

		std::cout << "Estimate: " << global_transform_2d << std::endl;
		// std::cout << "Matrix: " << global_transform << std::endl;

		//Map update
		double dist = (global_transform_2d[0] - last_up_pose[0]) * (global_transform_2d[0] - last_up_pose[0]) + (global_transform_2d[1] - last_up_pose[1]) * (global_transform_2d[1] - last_up_pose[1]);
		double d_angle = fabs(global_transform_2d[2] - last_up_pose[2]);

		if (d_angle > M_PI)
			d_angle = d_angle - M_PI * 2.0;
		else if (d_angle < -M_PI)
			d_angle = d_angle + M_PI * 2.0;
		d_angle = abs(d_angle);

		std::cout << "dist = " << dist << std::endl;
		std::cout << "d_angle = " << d_angle << std::endl;

		if (dist > 0.4 || d_angle > 0.05)
		{
			last_up_pose = global_transform_2d;
			std::cout << "Update" << std::endl;
			octomap::Pointcloud octo_pc;

			pcl2octopc(*input_cloud_tf, octo_pc);
			octomap::point3d sensor_origin(global_transform_2d[0], global_transform_2d[1], 0);

#ifdef USE_OCTOMAP
			octmaptree->insertPointCloudRays(octo_pc, sensor_origin); //REPLACE
			octomap2pcl(*octmaptree, *map_cloud);					  //get occupied nodes // REPLACE
#else
			occmap->insertPointCloudRays(octo_pc, sensor_origin);
			occmap2pcl(*occmap, *map_cloud);
#endif

			sensor_msgs::PointCloud2::Ptr out_msg = boost::make_shared<sensor_msgs::PointCloud2>();
			pcl::toROSMsg(*map_cloud, *out_msg);
			out_msg->header.frame_id = "map";
			map_cloud_publisher.publish(out_msg);
		}

		// tf2_ros::TransformBroadcaster br;
		// geometry_msgs::TransformStamped transform;
		// transform.child_frame_id = "cloud";
		// transform.header.frame_id = "map";
		// transform.header.stamp = ros::Time::now();
		// transform.transform.translation.x = global_transform_2d[0];
		// transform.transform.translation.y = global_transform_2d[1];
		// transform.transform.translation.z = 0;
		// transform.transform.rotation.w = q.w();
		// transform.transform.rotation.x = q.x();
		// transform.transform.rotation.y = q.y();
		// transform.transform.rotation.z = q.z();
		// br.sendTransform(transform);
		// ROS_INFO("Tf published ?");
	}

	//Process input cloud
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "my_loam3");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	//Get parameters
	double prob_occ;
	double prob_free;
	double prob_thres;
	int max_icp_it;
	std::string icp_method;
	if (!nh_private.getParam("map_resolution", g_map_res))
	{
		ROS_ERROR("set 'map_resolution' parameter!");
		ros::shutdown();
		exit(-1);
	}
	nh_private.param("prob_occ", prob_occ, 0.9);
	nh_private.param("prob_free", prob_free, 0.4);
	nh_private.param("prob_thres", prob_thres, 0.8);
	nh_private.param("input_voxel_size", input_voxel_res, 0.1);
	nh_private.param<std::string>("icp_method", icp_method, "icp");
	nh_private.param<int>("icp_iterations", max_icp_it, 10);

#ifdef USE_OCTOMAP
	octmaptree = new octomap::OcTree(g_map_res);
	octmaptree->setProbHit(prob_occ);
	octmaptree->setProbMiss(prob_free);
	octmaptree->setOccupancyThres(prob_thres);

#else
	occmap = new myslam::OccMap(g_map_res, 300);
	occmap->setOccupancyThres(prob_thres);
	occmap->setProbHit(prob_occ);
	occmap->setProbMiss(prob_free);
	occmap->setOccupancyThres(prob_thres);
	occmap->setStartOrigin(0.5, 0.5);
#endif

	ROS_WARN("map resolution -> %f", g_map_res);
	ROS_WARN("prob_occ -> %f", prob_occ);
	ROS_WARN("prob_free -> %f", prob_free);
	ROS_WARN("prob_thres -> %f", prob_thres);
	ROS_WARN("input_voxel_size -> %f", input_voxel_res);

	map_cloud = boost::make_shared<PointCloudT>();

	std::string cloud_topic = nh.resolveName("cloud");
	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 5, cloud_callback);
	map_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 5);

	ros::Rate loop(20);
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// ros::waitForShutdown();
	// exit(0);

	while (ros::ok())
	{
		// ros::spinOnce();

		// g_lock.lock();
		// pcl::transformPointCloud(*current_cloud,*current_cloud,global_transform);
		// g_lock.unlock();

		// octomap::Pointcloud octo_pc;

		// g_lock.lock();
		// pcl2octopc(*current_cloud,octo_pc);
		// g_lock.unlock();
		// octomap::point3d sensor_pose(global_transform(0,3),global_transform(1,3),0);
		// // octomap::point3d sensor_pose(0,0,0);
		// // octmaptree->insertPointCloudRays(octo_pc,sensor_pose);

		// octomap2pcl(*octmaptree, *map_cloud); //get occupied nodes

		// sensor_msgs::PointCloud2::Ptr out_msg = boost::make_shared<sensor_msgs::PointCloud2>();
		// pcl::toROSMsg(*map_cloud, *out_msg);
		// out_msg->header.frame_id = "map";
		// map_cloud_publisher.publish(out_msg);

		// ROS_WARN("Updating map...");

		static tf2_ros::TransformBroadcaster broad;
		geometry_msgs::TransformStamped tf;
		tf.header.stamp = ros::Time::now();
		tf.header.frame_id = "map";
		tf.child_frame_id = "cloud";
		tf.transform.translation.x = global_transform(0, 3);
		tf.transform.translation.y = global_transform(1, 3);
		tf.transform.translation.z = 0;
		Eigen::Quaterniond q;
		q = Eigen::AngleAxisd(global_transform_2d[2], Eigen::Vector3d::UnitZ());
		tf.transform.rotation.w = q.w();
		tf.transform.rotation.x = q.x();
		tf.transform.rotation.y = q.y();
		tf.transform.rotation.z = q.z();

		std::cout << "x = " << tf.transform.translation.x << std::endl;
		std::cout << "y = " << tf.transform.translation.y << std::endl;
		broad.sendTransform(tf);

		loop.sleep();
	}
}
