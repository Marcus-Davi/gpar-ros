/* Author : Marcus Forte <davi2812@dee.ufc.br> */

#include <ros/ros.h> //General Ros stuff

// PCL
#include <pcl/point_types.h> // PCL types .. i.e PointXYZ
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
#include <std_msgs/String.h> //receive messages to manipulate cloud 
#include <sensor_msgs/PointCloud2.h> // Process PC2 message
#include "geometry_msgs/TransformStamped.h"

// TF2
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.h" // Allows direct transform application on ros message4

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT; // Define a templated type of pointcloud

ros::Publisher map_cloud_publisher;
PointCloudT::Ptr map_cloud;
Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
geometry_msgs::Pose2D update_pose;

// Debug Vis
static pcl::visualization::PCLVisualizer::Ptr viewer;
int vp0,vp1;


bool first_cloud = true;

// Parameters
float icp_voxel_res = 0.06;
float map_voxel_res = 0.03;


void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
// ROS_INFO("got cloud!");

PointCloudT::Ptr input_cloud = boost::make_shared<PointCloudT>();
PointCloudT::Ptr input_cloud_voxel = boost::make_shared<PointCloudT>();

pcl::fromROSMsg<pcl::PointXYZ>(*pc_msg,*input_cloud);


static pcl::VoxelGrid<pcl::PointXYZ> voxel;	
voxel.setLeafSize(icp_voxel_res,icp_voxel_res,icp_voxel_res); //preserve voxel

static pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud(input_cloud);
pass.setFilterFieldName("x");
pass.setFilterLimits(0.2,8); 
pass.filter(*input_cloud);

voxel.setInputCloud(input_cloud);
voxel.filter(*input_cloud_voxel);


if(first_cloud){
	map_cloud = boost::make_shared<PointCloudT>();
	*map_cloud = *input_cloud;
	first_cloud = false;
	update_pose.x = 0;
	update_pose.y = 0;
	update_pose.theta = 0;
	voxel.setInputCloud(map_cloud);
	voxel.setLeafSize(map_voxel_res,map_voxel_res,map_voxel_res); //preserve voxel
	voxel.filter(*map_cloud);
	ROS_WARN("Forming Map");
	//Update
	// *previous_cloud = *input_cloud;
	
	
}




PointCloudT::Ptr aligned_cloud = boost::make_shared<PointCloudT>();
static pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
static pcl::registration::TransformationEstimation2D<pcl::PointXYZ,pcl::PointXYZ>::Ptr te_2d (new pcl::registration::TransformationEstimation2D<pcl::PointXYZ,pcl::PointXYZ>);
// pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;

//Incremental alignment
icp.setInputTarget(map_cloud);
icp.setInputSource(input_cloud_voxel);
icp.setTransformationEstimation(te_2d);

icp.setMaxCorrespondenceDistance(0.1);
// icp.setMaximumOptimizerIterations(5);
// icp.setRotationEpsilon(2);
icp.setMaximumIterations(90);
icp.setTransformationEpsilon(1e-7);
icp.align(*aligned_cloud,global_transform);



// viewer.removeAllPointClouds();
// viewer.addPointCloud(previous_cloud,"target",0);
// viewer.addPointCloud(input_cloud,"source",vp0);
// viewer.addPointCloud(aligned_cloud,"aligend",vp1);
// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,0,0,"target");
// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"source");
// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,1,"aligend");


//Update pose
 Eigen::Matrix4f current_transform = icp.getFinalTransformation();

std::cout << "Score: " << icp.getFitnessScore() << std::endl;


// global_transform = global_transform*current_transform;
global_transform = current_transform;

std::cout << "Current: " << current_transform << std::endl;
std::cout << "Global: " << global_transform << std::endl;


Eigen::Vector3f translation = global_transform.block<3,1>(0,3);
Eigen::Matrix3f rotation = global_transform.block<3,3>(0,0);
Eigen::Vector3f rot_vec = rotation.eulerAngles(0,1,2);
Eigen::Quaternionf rot_quat(rotation);

// std::cout << "rotation = " << rot_vec << std::endl << std::endl;

float dx = translation[0] - update_pose.x;
float dy = translation[1] - update_pose.y;
float dt = fabs(rot_vec[2] - update_pose.theta);

float dist = sqrt(dx*dx - dy*dy);

if(dist > 0.3 || dt > 0.07){
	/// MAPPING

	//Transform full cloud
	pcl::transformPointCloud(*input_cloud,*input_cloud,icp.getFinalTransformation());
	*map_cloud += *input_cloud;
	voxel.setInputCloud(map_cloud);
	voxel.setLeafSize(map_voxel_res,map_voxel_res,map_voxel_res); //preserve voxel
	voxel.filter(*map_cloud);
	
	update_pose.x = translation[0];
	update_pose.y = translation[1];
	update_pose.theta = rot_vec[2];
	ROS_WARN("Updating Map");
}




//Update transform
static tf2_ros::TransformBroadcaster br; //tf Broadcaster
geometry_msgs::TransformStamped tf_transform;
tf_transform.header.stamp = pc_msg->header.stamp;
tf_transform.header.frame_id = "map";
tf_transform.child_frame_id = "cloud";
tf_transform.transform.translation.x = global_transform(0,3);
tf_transform.transform.translation.y = global_transform(1,3);
tf_transform.transform.translation.z = 0;
tf_transform.transform.rotation.w = rot_quat.w();
tf_transform.transform.rotation.x = rot_quat.x();
tf_transform.transform.rotation.y = rot_quat.y();
tf_transform.transform.rotation.z = rot_quat.z();
br.sendTransform(tf_transform);

//Update
// *previous_cloud = *input_cloud;


sensor_msgs::PointCloud2::Ptr out_msg = boost::make_shared<sensor_msgs::PointCloud2>();
pcl::toROSMsg(*map_cloud,*out_msg);
out_msg->header.stamp = pc_msg->header.stamp;
out_msg->header.frame_id = "map";
map_cloud_publisher.publish(out_msg);



}


int main(int argc,char** argv){
	ros::init(argc,argv,"slam_icp");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	
	std::string cloud_topic = nh.resolveName("cloud");

	ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic,100,cloud_callback);
	map_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("map_cloud",100);


	// viewer = 
	// viewer.createViewPort(0,1,0.5,1,vp0);
	// viewer.createViewPort(0.5,0,1,1,vp1);
	// viewer.addCoordinateSystem(1,"ref",0);

	// viewer.close();
	while(ros::ok()){


		// viewer.spinOnce();
		ros::spinOnce();
	}
	


}
