#include "image_transport/camera_subscriber.h"
#include "image_transport/publisher.h"
#include "ros/forwards.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/transport_hints.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>


#include <image_transport/image_transport.h>


image_transport::Publisher cam_pub;

void cam_callback(const sensor_msgs::Image::ConstPtr& img){
ROS_INFO("got img");


sensor_msgs::Image::Ptr pub_img = boost::make_shared<sensor_msgs::Image>();
*pub_img = *img;

cam_pub.publish(pub_img);

}


int main(int argc,char** argv){
		ros::init(argc,argv,"camera_get");
		ros::NodeHandle nh;
		ros::NodeHandle nh_private("~");
		image_transport::ImageTransport it(nh);

		std::string topic = nh.resolveName("image"); //importantissimo pra usar o image_transport

		ROS_INFO("sub to topic : %s",topic.c_str());

		//image_transport::TransportHints hints("compressed",ros::TransportHints().udp());
		
		image_transport::Subscriber cam_sub = it.subscribe(topic,1,cam_callback); //remap
		
		

//		image_transport::CameraSubscriber cam_sub = it.subscribeCamera("usb_cam",1,cam_callback);
		//ros::Subscriber cam_sub = nh.subscribe("usb_cam/image_raw",5,cam_callback,ros::TransportHints().udp());

//		cam_pub = nh.advertise<sensor_msgs::Image>("mycam",1);
		
		cam_pub = it.advertise("mycam",1);



		ros::spin();
}
