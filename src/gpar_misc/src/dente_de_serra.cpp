#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char **argv){	
	int freq = 1,duty = 0,auxiliar = 1;
	std_msgs::Int32 duty_cycle ;

	duty_cycle.data = 0;
	
	ros::init (argc,argv,"dente_de_serra");
	
	ros::NodeHandle n("~");
	ros::Publisher info = n.advertise<std_msgs::Int32>("dutycycle",100);
		
	
	while(ros::ok()){
		ros::Rate loop_rate(freq);
		n.getParam("frequencia",freq); 

		if(auxiliar)
			duty_cycle.data++;
		else
			duty_cycle.data--;

		if(duty_cycle.data == 100)
			auxiliar = 0;
		else if(duty_cycle.data == -100)
			auxiliar = 1;
		
		ROS_INFO("%d %d",duty_cycle.data,freq);	// Só para verificarmos os valores
		info.publish(duty_cycle);
		
	loop_rate.sleep();		

	}
return 0;
}
