#include "ros/node_handle.h"
#include "ros/ros.h"
#include "std_msgs/String.h" //Vem da plaquinha
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/QuaternionStamped.h"

// Kalman
#include "gpar_kalman/Kalman.h"
#include "gpar_kalman/ModelFunctions.h"

// Message Filters
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

geometry_msgs::Vector3Stamped acc, mag, gyr;
// bool got_acc = false;
// bool got_gyr = false;
// bool got_mag = false;

// void acc_callback(const geometry_msgs::Vector3::ConstPtr& msg){
// acc = *msg;
// got_acc = true;
// }

// void gyr_callback(const geometry_msgs::Vector3::ConstPtr& msg){
// gyr = *msg;
// got_gyr = true;
// }

// void mag_callback(const geometry_msgs::Vector3::ConstPtr& msg){
// mag = *msg;
// got_mag = true;
// }

bool hasData = false;
void imuCallback(const geometry_msgs::Vector3StampedConstPtr &acc_,
                 const geometry_msgs::Vector3StampedConstPtr &gyr_,
                 const geometry_msgs::Vector3StampedConstPtr &mag_)
{
// ROS_INFO("Sync!");
acc = *acc_;
gyr = *gyr_;
mag = *mag_;
hasData = true;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "kalman_k64f");

    //TODO Tornar topicos inscritos parametrizados

    std::string parent_frame;
    std::string child_frame;

    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    if (!nh.getParam("parent_frame", parent_frame))
    {
        parent_frame = "map";
        ROS_WARN("'parent_frame' parameter not set. setting to %s", parent_frame.c_str());
    }

    if (!nh.getParam("child_frame", child_frame))
    {
        child_frame = "imu";
        ROS_WARN("'child_frame' parameter not set. setting to %s", child_frame.c_str());
    }

    message_filters::Subscriber<geometry_msgs::Vector3Stamped> acc_sub(n, "k64f_imu/accelerations", 1);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> gyr_sub(n, "k64f_imu/angular_vels", 1);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> mag_sub(n, "k64f_imu/magnetic_field", 1);
    // message_filters::TimeSynchronizer<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> sync(acc_sub, gyr_sub, mag_sub, 10);
     typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped,geometry_msgs::Vector3Stamped> MySyncPolicy;
    
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),acc_sub,gyr_sub,mag_sub);
    sync.registerCallback(boost::bind(&imuCallback, _1, _2, _3));
    // ros::Subscriber sub_a = n.subscribe("k64f_imu/accelerations",100,acc_callback);
    // ros::Subscriber sub_g = n.subscribe("k64f_imu/angular_vels",100,gyr_callback);
    // ros::Subscriber sub_m = n.subscribe("k64f_imu/magnetic_field",100, mag_callback);
    ros::Publisher pub_q = n.advertise<geometry_msgs::QuaternionStamped>("kalman_quaternion",10);

    // EKF Settings

    //Covariances
    double Qn[4 * 4] = {
        0.001, -0.0003, 0.0003, 0.0003,
        -0.0003, 0.0001, -0.0001, -0.0001,
        0.0003, -0.0001, 0.0001, 0.0001,
        0.0003, -0.0001, 0.0001, 0.0001}; //3x3 n x n

    double Rn[6 * 6] = {
        0.01, 0, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0, 0,
        0, 0, 0.01, 0, 0, 0,
        0, 0, 0, 5, 0, 0,
        0, 0, 0, 0, 2, 0,
        0, 0,  0, 0, 0, 0.6,
    }; //3x3 out x out

    double X0[4] = {1, 0, 0, 0};

    Kalman::EKF Filter(4, 3, 6);
    Filter.SetQn(Qn);
    Filter.SetRn(Rn);
    Filter.SetX0(X0);
    Filter.SetStateFunction(AttitudeEstimation::StateFunction);
    Filter.SetStateJacobian(AttitudeEstimation::StateJacobian);
    Filter.SetMeasurementFunction(AttitudeEstimation::MeasurementFunction);
    Filter.SetMeasurementJacobian(AttitudeEstimation::MeasurementJacobian);

    

    Kalman::VectorKalman states(4);

    geometry_msgs::Quaternion q;
    geometry_msgs::QuaternionStamped qs;
    static tf2_ros::TransformBroadcaster br; //tf Broadcaster
    geometry_msgs::TransformStamped imuTransform;

    ROS_INFO("Publising rotation ...");
    ROS_INFO("Publishing TF %s -> %s", parent_frame.c_str(), child_frame.c_str());

    Kalman::VectorKalman input(3);
    Kalman::VectorKalman measurement(6);

    while (ros::ok())
    {

        if (!(hasData))
        {
            ros::spinOnce();
            continue;
        }

        hasData = false;
        mag_field = AttitudeEstimation::GetMagField(mag.vector);

        input[0] = gyr.vector.x;
        input[1] = gyr.vector.y;
        input[2] = gyr.vector.z;

        measurement[0] = acc.vector.x;
        measurement[1] = acc.vector.y;
        measurement[2] = acc.vector.z;

        measurement[3] = mag.vector.x;
        measurement[4] = mag.vector.y;
        measurement[5] = mag.vector.z;

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

        float norm = sqrt(states[0] * states[0] + states[1] * states[1] + states[2] * states[2] + states[3] * states[3]);

        q.w = states[0] / norm;
        q.x = states[1] / norm;
        q.y = states[2] / norm;
        q.z = states[3] / norm;

        //pub_q.publish(q);

        imuTransform.header.stamp = ros::Time::now();
        imuTransform.header.frame_id = parent_frame; //"map" ou "odom"
        imuTransform.child_frame_id = child_frame;   //"cloud" ou "imu"

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
