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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT; // Define a templated type of pointcloud

// Map Publishing

ros::Publisher map_cloud_publisher;

// Map Parameters
double input_voxel_res;
octomap::OcTree *octmaptree;
double g_map_res;

//Global transform
Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
Eigen::Vector3d global_transform_2d = Eigen::Vector3d::Zero();

//Odometry variable
PointCloudT::Ptr current_cloud;
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
	ROS_INFO("LASER");
	if (first_map)
	{
		octomap::Pointcloud first_pc;
		octomap::point3d origin(0, 0, 0);
		g_lock.lock();
		pcl2octopc(*input_cloud, first_pc);
		g_lock.unlock();

		octmaptree->insertPointCloudRays(first_pc, origin);
		first_map = false;
	}
	else
	{
		pcl::PassThrough<pcl::PointXYZ> remover;
		remover.setInputCloud(input_cloud);
		remover.setFilterFieldName("x");
		remover.setFilterLimits(-0.1,0.1);
		remover.setNegative(true);
		remover.filter(*input_cloud);

		pcl::transformPointCloud(*input_cloud,*input_cloud,global_transform);

		Eigen::Matrix<double,1,2> dm;
		Eigen::Matrix<double,2,3> jac;
		Eigen::Matrix3d Hessian = Eigen::Matrix3d::Identity(3,3);
		Eigen::Vector3d dtr = Eigen::Vector3d::Zero();


		double funval;
		for (int i = 0; i < input_cloud->size(); ++i)
		{
			funval = 1 - myslam::mapAccess(*octmaptree,input_cloud->points[i]);
			dm = myslam::mapGradient<1,2>(*octmaptree,input_cloud->points[i]);
			jac = myslam::modelGradient<2,3>(input_cloud->points[i],global_transform_2d);
			// std::cout << "funval = " << funval << std::endl;
			// std::cout << "dm = " << dm << std::endl;
			// std::cout << "jac = " << jac << std::endl;
			dtr = dtr + (dm*jac).transpose()*funval;
			Hessian = Hessian + (dm*jac).transpose()*(dm*jac);

		}
			Eigen::Vector3d searchdir = Eigen::Vector3d::Zero();
			if(Hessian(0,0) != 0 && Hessian(1,1) != 0){
			searchdir = Hessian.inverse()*dtr;
			std::cout << "Search dir = " << searchdir << std::endl;
			}
			global_transform_2d = global_transform_2d + searchdir;
			global_transform(0,3) = global_transform_2d[0];
			global_transform(1,3) = global_transform_2d[1];
			std::cout << "Estiamte: " << global_transform_2d << std::endl;




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

	octmaptree = new octomap::OcTree(g_map_res);
	octmaptree->setProbHit(prob_occ);
	octmaptree->setProbMiss(prob_free);
	octmaptree->setOccupancyThres(prob_thres);

	ROS_WARN("map resolution -> %f", g_map_res);
	ROS_WARN("prob_occ -> %f", prob_occ);
	ROS_WARN("prob_free -> %f", prob_free);
	ROS_WARN("prob_thres -> %f", prob_thres);
	ROS_WARN("input_voxel_size -> %f", input_voxel_res);

	map_cloud = boost::make_shared<PointCloudT>();

	std::string cloud_topic = nh.resolveName("cloud");
	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 100, cloud_callback);
	map_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 100);

	ros::Rate r(1);
	ros::AsyncSpinner spinner(1);
	spinner.start();

	while (ros::ok())
	{
		// ros::spinOnce();

		octomap2pcl(*octmaptree, *map_cloud); //get occupied nodes

		sensor_msgs::PointCloud2::Ptr out_msg = boost::make_shared<sensor_msgs::PointCloud2>();
		pcl::toROSMsg(*map_cloud, *out_msg);
		out_msg->header.frame_id = "map";
		map_cloud_publisher.publish(out_msg);

		ROS_WARN("Updating map...");

		r.sleep();
	}
}
