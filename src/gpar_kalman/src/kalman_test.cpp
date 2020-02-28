#include "ros/ros.h"
#include "std_msgs/String.h" //Vem da plaquinha

#include "Kalman.h"






int main(int argc, char** argv){
ros::init(argc,argv, "kalman_test");

ros::NodeHandle n;


ros::spin();

}
