#include <ros/ros.h>
#include <wiringPi.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>



int pinA = 25;
int pinB = 24;


int count = 0;

bool reset_count(std_srvs::Empty::Request& req, std_srvs::Empty::Response &res){
		ROS_INFO("serv");
	count = 0;
	return true;

}


void enc_callback(){
		if(digitalRead(pinB) == HIGH)
				count ++;
		else
				count--;


}

int main(int argc,char** argv){
		ros::init(argc,argv,"encoder_node");
		ros::NodeHandle nh;
		ros::NodeHandle nh_private("~");

		wiringPiSetup();


		if (! nh_private.getParam("pin_a",pinA) ) {
				ROS_WARN("setting default pinA = %d",pinA);
		} else {
				ROS_INFO("pinA = %d",pinA);
		}
		if (! nh_private.getParam("pin_b",pinB) ) {
				ROS_WARN("setting default pinB = %d",pinB);
		} else {
				ROS_INFO("pinB = %d",pinB);
		}


		pinMode(pinA,INPUT);
		pinMode(pinB,INPUT);

		wiringPiISR(pinA,INT_EDGE_RISING,enc_callback);


		ros::Rate r(10);

		ros::Publisher pos_pub = nh.advertise<std_msgs::Int32>("enc_pos",1);

		ros::ServiceServer reset_serv = nh.advertiseService("reset_count",reset_count);


		std_msgs::Int32 msg;
		while(ros::ok()){
				msg.data = count;
				pos_pub.publish(msg);
				r.sleep();

				ros::spinOnce();
		}

		return 0;
}


