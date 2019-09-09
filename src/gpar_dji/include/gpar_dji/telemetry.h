

#ifndef TELEMETRY_H
#define TELEMETRY_H



//ROS INCLUDES
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
//DJI SDK







//Callback Prototypes
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);



#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))


























#endif // TELEMETRY_H
