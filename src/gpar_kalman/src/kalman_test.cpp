#include "ros/ros.h"
#include "std_msgs/String.h" //Vem da plaquinha

#include "Kalman.h"
#include "gpar_kalman/ModelFunctions.h"


#define iterations 1000000

int main(int argc, char** argv){

ros::init(argc,argv, "kalman_test");

ros::NodeHandle n;

//My stuff
MD::Quaternion q(1,2,3,4);
Vec3 a_(1,2,3);
Vec3 rotated_a_;

//eigen stuff
Eigen::Quaternionf q1(1,2,3,4);
Eigen::Quaternionf qa(0,1,2,3);

Eigen::Quaternionf qa_rotated;



ros::Time begin = ros::Time::now();

for(unsigned int i=0;i<iterations;++i)
rotated_a_ = q.RotateFrame(a_);


ros::Time end = ros::Time::now();

std::cout << "my = " << end-begin << std::endl;



//Certamente há forma mais eficiente pro EIGEN
begin = ros::Time::now();

for(unsigned int i=0;i<iterations;++i)
qa_rotated = q1.inverse()*qa*q1;

 end = ros::Time::now();

 std::cout << "eigen = " << end-begin << std::endl;

std::cout << "myrot : " << rotated_a_.x << "," << rotated_a_.y << "," << rotated_a_.z << std::endl;
std::cout << "eigen : " << qa_rotated.x() << "," << qa_rotated.y() << "," << qa_rotated.z() << std::endl;




ros::spin();

}
