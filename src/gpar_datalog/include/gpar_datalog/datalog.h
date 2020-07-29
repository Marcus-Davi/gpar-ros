#include <ros/ros.h>
#include <rosbag/bag.h>



namespace gpar {


		//TODO -> Desafio : como criar uma callback autonomamente p/ cada novo topico a ser gravado ?
		// Como descobrir o type pela msg ?

class datalog {
		public:
				datalog();
				~datalog();


				

						

		private:
				ros::NodeHandle nh;

};



}
