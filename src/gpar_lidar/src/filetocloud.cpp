/* Author : Marcus Forte <davi2812@dee.ufc.br> */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>


#include <pcl_conversions/pcl_conversions.h>






int main(int argc,char** argv){
    ros::init(argc,argv,"filetocloud");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    




    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("file_cloud",1,true);

    std::string cloud_file;

    if (! nh_private.getParam("cloudfile",cloud_file)){
        ROS_FATAL("please, use parameter 'cloudfile' to specify .pcd cloud path");

        ros::shutdown();
        exit(-1);
    }

    // PATH OK

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
int k;
    if(pcl::io::loadPCDFile(cloud_file,*cloud)){
        ROS_FATAL("could not open .pcd file.");
        ros::shutdown();
        exit(-1);
    }
    

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud,msg);
    msg.header.frame_id = "cloud";

    cloud_pub.publish(msg);

    ROS_INFO("publishing cloud!");

    while(ros::ok()){


    }    




    return 0;
}