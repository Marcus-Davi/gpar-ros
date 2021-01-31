#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "tf2_ros/transform_broadcaster.h"


sensor_msgs::Imu imudata;
// sensor_msgs::


void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
    imudata = *msg;
    //ROS_INFO("android imu..");
    static tf2_ros::TransformBroadcaster br;
    static geometry_msgs::TransformStamped imuTransform;

    imuTransform.header.stamp = ros::Time::now();
    imuTransform.header.frame_id = "map";
    imuTransform.child_frame_id = "android";

    imuTransform.transform.translation.x = 0;
    imuTransform.transform.translation.y = 0;
    imuTransform.transform.translation.z = 0;
    imuTransform.transform.rotation = imudata.orientation;

    br.sendTransform(imuTransform);


    
}

// void mag_callback



int main(int argc, char** argv){

ros::init(argc,argv, "android_tf");

//TODO Tornar topicos inscritos parametrizados
ROS_INFO("android tf");
ros::NodeHandle n;
ros::Subscriber sub_imu = n.subscribe("/android/imu",100,imu_callback);
//ros::Subscriber sub_m = n.subscribe("android/magnetic_field",100, mag_callback);
//
ros::spin();



}
