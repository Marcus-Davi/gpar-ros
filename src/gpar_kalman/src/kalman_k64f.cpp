#include "ros/ros.h"
#include "std_msgs/String.h" //Vem da plaquinha
#include "geometry_msgs/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/QuaternionStamped.h"


#include "gpar_kalman/Kalman.h"
#include "gpar_kalman/ModelFunctions.h"


geometry_msgs::Vector3 acc,mag,gyr;
bool got_acc = false;
bool got_gyr = false;
bool got_mag = false;

void acc_callback(const geometry_msgs::Vector3::ConstPtr& msg){
acc = *msg;
got_acc = true;
}

void gyr_callback(const geometry_msgs::Vector3::ConstPtr& msg){
gyr = *msg;
got_gyr = true;
}

void mag_callback(const geometry_msgs::Vector3::ConstPtr& msg){
mag = *msg;
got_mag = true;
}


int main(int argc, char** argv){

ros::init(argc,argv, "kalman_");

//TODO Tornar topicos inscritos parametrizados

ros::NodeHandle n;
ros::Subscriber sub_a = n.subscribe("k64f_imu/accelerations",100,acc_callback);
ros::Subscriber sub_g = n.subscribe("k64f_imu/angular_vels",100,gyr_callback);
ros::Subscriber sub_m = n.subscribe("k64f_imu/magnetic_field",100, mag_callback);
ros::Publisher pub_q = n.advertise<geometry_msgs::QuaternionStamped>("kalman_quaternion",10);

ros::Rate r(50);

// EKF Settings

    //Covariances
    double Qn[4*4] = {
    		0.001, -0.0003, 0.0003, 0.0003,
    		-0.0003,0.0001,-0.0001,-0.0001,
    		0.0003,-0.0001,0.0001,0.0001,
    		0.0003,-0.0001,0.0001,0.0001
    }; //3x3 n x n

    double Rn[6*6] = {
    		0.1,0,0,0,0,0,
    		0,0.1,0,0,0,0,
    		0,0,0.1,0,0,0,
    		0,0,0,2,0,0,
    		0,0,0,0,2,0,
    		0,0,0,0,0,3,
    }; //3x3 out x out

    double X0[4] = {1,0,0,0};



Kalman::EKF Filter(4,3,6);
Filter.SetQn(Qn);
Filter.SetRn(Rn);
Filter.SetX0(X0);
Filter.SetStateFunction(AttitudeEstimation::StateFunction);
Filter.SetStateJacobian(AttitudeEstimation::StateJacobian);
Filter.SetMeasurementFunction(AttitudeEstimation::MeasurementFunction);
Filter.SetMeasurementJacobian(AttitudeEstimation::MeasurementJacobian);

double input[3];
double measurement[6];

double states[4];

geometry_msgs::Quaternion q;
geometry_msgs::QuaternionStamped qs;
static tf2_ros::TransformBroadcaster br; //tf Broadcaster
geometry_msgs::TransformStamped imuTransform;

ROS_INFO("Publising rotation ...");

while(ros::ok()){



    if (!(got_acc && got_mag && got_gyr)){
    	ros::spinOnce();
    	continue;
    }
    mag_field = AttitudeEstimation::GetMagField(mag);
    got_acc = false;
    got_mag = false;
    got_gyr = false;

    input[1] = gyr.x;
    input[0] = -gyr.y;
    input[2] = gyr.z;

    measurement[1] = acc.x;
    measurement[0] = -acc.y;
    measurement[2] = acc.z;

    measurement[4] = mag.x;
    measurement[3] = -mag.y;
    measurement[5] = mag.z;

//input[1] = gyr.x;
//input[0] = -gyr.y;
//input[2] = gyr.z;
//
//measurement[1] = acc.x;
//measurement[0] = -acc.y;
//measurement[2] = acc.z;
//
//measurement[4] = mag.x;
//measurement[3] = -mag.y;
//measurement[5] = mag.z;

Filter.Predict(input);
Filter.Update(measurement);
Filter.GetEstimatedStates(states);


float norm = sqrt(states[0]*states[0]+states[1]*states[1]+states[2]*states[2]+states[3]*states[3]);

q.w = states[0]/norm;
q.x = states[1]/norm;
q.y = states[2]/norm;
q.z = states[3]/norm;

//pub_q.publish(q);

imuTransform.header.stamp = ros::Time::now();
imuTransform.header.frame_id = "map"; //"map" ou "odom"
imuTransform.child_frame_id = "k64f"; //"cloud" ou "imu"

imuTransform.transform.translation.x = 0;
imuTransform.transform.translation.y = 0;
imuTransform.transform.translation.z = 0; 
imuTransform.transform.rotation = q;

br.sendTransform(imuTransform);

qs.quaternion = q;
qs.header.stamp = imuTransform.header.stamp;
pub_q.publish(qs);



ros::spinOnce();

}

}
