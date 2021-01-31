#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
ros::Publisher cloud_pub2; 
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
		sensor_msgs::PointCloud2 out_msg;

		PointCloud cloud;
		pcl::fromROSMsg(*msg,cloud);

		// Iterator melhor ?
		for (int i=0;i<cloud.size();++i){
				cloud.points[i].x += 1;
		}


		pcl::toROSMsg(cloud,out_msg);
 
		cloud_pub2.publish(out_msg);


}

int main(int argc,char** argv){
		ros::init(argc,argv,"point_test");
		ros::NodeHandle nh;


		ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pontos",1);
		cloud_pub2		= nh.advertise<sensor_msgs::PointCloud2>("pontos_out",1);

		ros::Subscriber cloud_sub = nh.subscribe("pontos",1,callback);

		PointCloud a;
		a.header.frame_id = "map";
		pcl::PointXYZ pt(0,0,0);

		ros::Rate r(1);

		sensor_msgs::PointCloud2 msg;

		while(ros::ok()){


				pt.z += 0.05;
				a.push_back(pt);

				ROS_INFO("size = %ld",a.size());

				pcl::toROSMsg(a,msg);


				cloud_pub.publish(msg);

				ros::spinOnce();
				r.sleep();
		}


}

