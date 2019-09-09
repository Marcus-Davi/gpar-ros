#include <ros/ros.h>
#include <rosbag/bag.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>



tf::TransformListener* tf_;

void pc_callback(const sensor_msgs::PointCloud2::ConstPtr& pc){


}


int main(int argc, char**argv){


	/*ros::init(argc, argv, "bag_log");
	ros::NodeHandle nh;
	ROS_INFO("Logging file...");

	tf_ = new tf::TransformListener(nh, ros::Duration(3.0));
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub;
	tf::MessageFilter<sensor_msgs::PointCloud2>* tf_filter;

	sub.subscribe(nh, input_cloud_node, 10);
	tf_filter = new tf::MessageFilter<sensor_msgs::PointCloud2>(sub, *tf_, fixed_frame_, 10);
	tf_filter->registerCallback(boost::bind(pc_callback, _1));*/

  
	ros::spin();

	return 0;

}




