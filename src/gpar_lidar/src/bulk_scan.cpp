/* Author : Marcus Forte <davi2812@dee.ufc.br> */

#include "ros/subscriber.h"
#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>



#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>   // necessary because of custom point type


#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <std_msgs/UInt8.h>

// Ativa ou nao o merge
#include <std_srvs/SetBool.h>


ros::Publisher merged_cloud;

std::string obj_frame;

tf2_ros::TransformListener *tf_listener;
tf2_ros::Buffer *tf_buffer;


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
float vel;
float x0 = 0;
ros::Time t0;
bool first = false;

bool start_merge = false;

// Interval
// float x_min;
// float x_max;


// Nuvem Global
PointCloudT::Ptr cloud_object = boost::make_shared<PointCloudT>();





bool merge_service(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res){
		start_merge = req.data;
		if(start_merge)
				res.message = "Merging!";
		else
				res.message = "Waiting merge.";


		res.success = true;		
		return true;
}


//TODO usar algum message filter p/ quadros ? 
void top_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg){
		if(start_merge){
				ROS_WARN("BS: merging... %ld points",cloud_object->size());
		} else {
				ROS_WARN("BS: waiting..");
				cloud_object->clear(); // Verificar se ja ta vazio
		}

		// ROS_INFO("BulkScan : synced");

		if(!start_merge)
				return;


		PointCloudT::Ptr cloud_top = boost::make_shared<PointCloudT>();
		PointCloudT::Ptr cloud_map = boost::make_shared<PointCloudT>();

		PointCloudT::Ptr cloud_map_output = boost::make_shared<PointCloudT>();



		pcl::fromROSMsg(*cloud_msg,*cloud_top);



		// Transforma

		try { 
				//geometry_msgs::TransformStamped tf = tf_buffer->lookupTransform("map",pc->header.frame_id,ros::Time(0));

				pcl_ros::transformPointCloud("map",*cloud_top,*cloud_map,*tf_buffer);


				//*cloud_map = *cloud_top; // LEGACY

				// O "map" tem origem imediatamente abaixo do cloud_top
				geometry_msgs::TransformStamped tf_map_obj = tf_buffer->lookupTransform("map",obj_frame,ros::Time(0));
				


				/* A Ideia :
				   1) Transforma nuvem "top" segundo frame "map". chamo de "cloud_map"
				   2) Um quadro móvel, "obj", se desloca junto com objeto a ser escaneado.
				   3) Translada nuvem "cloud_obj" junto com objeto. A "cloud_object" contem todo histórico de medidas
				   4) Grava/agrega/soma/junta a nuvem "cloud_map" em "cloud_object". chamo de "cloud_object"
				   3) 

*/


				//Translada
				pcl_ros::transformPointCloud(*cloud_map,*cloud_map,tf_map_obj.transform);

				// Funde. Por algum motivo a nuvem tá sendo duplicada no map e no obj
				*cloud_object += *cloud_map;


		} catch(tf2::TransformException &ex) {
				ROS_WARN("%s", ex.what());
		}

		// Apenas espelhar
		geometry_msgs::Transform mirror_tf ;
		tf2::Quaternion q_;
		q_.setRPY(0,0,M_PI);
		mirror_tf.rotation = tf2::toMsg(q_);
		mirror_tf.translation.x = 0;
		mirror_tf.translation.y = 0;
		mirror_tf.translation.z = 0;

		//pcl_ros::transformPointCloud(*cloud_object,*cloud_object,mirror_tf);


		sensor_msgs::PointCloud2Ptr cloud_out_msg = boost::make_shared<sensor_msgs::PointCloud2>();

		pcl::toROSMsg(*cloud_object,*cloud_out_msg);



		cloud_out_msg->header.frame_id = "map"; //Global/Fixed frame
		cloud_out_msg->header.stamp = ros::Time::now();


		merged_cloud.publish(cloud_out_msg);

		// Merge 
}




int main(int argc, char **argv)
{
		ros::init(argc, argv, "bulk_scan");
		ros::NodeHandle nh;
		ros::NodeHandle private_nh("~");

		// Frame of the object to be tracked
		if (! private_nh.getParam("obj_frame",obj_frame) ) {
				obj_frame = "obj";
		}
		ROS_WARN("obj_frame set to %s",obj_frame.c_str());

		std::string cloud_topic = nh.resolveName("cloud_topic");

		ROS_WARN("bulk scanner subscribed to -> %s",cloud_topic.c_str());
		ros::ServiceServer service = private_nh.advertiseService("start_merging",merge_service);



		//Usar message filter ? 
		ros::Subscriber cloud_sub = nh.subscribe(cloud_topic,1,top_callback);






		merged_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_merged",10);


		tf_buffer = new tf2_ros::Buffer;
		tf_listener = new tf2_ros::TransformListener(*tf_buffer);



		ros::spin();


		delete tf_buffer;
		delete tf_listener;

		return 0;
}


