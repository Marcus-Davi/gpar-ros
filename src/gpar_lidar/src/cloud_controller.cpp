/* Author : Marcus Forte <davi2812@dee.ufc.br> */

#include <ros/ros.h>
#include <std_msgs/UInt16.h>

// #include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>

#include <sensor_msgs/PointCloud2.h>
#include <sick_ldmrs_msgs/sick_ldmrs_point_type.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>
#include <pcl_ros/impl/transforms.hpp>   // necessary because of custom point type

#include <gpar_lidar/Command.h>

#include <fstream> //file save
#include <sys/stat.h>
#include <ctime>
using namespace std;

std::vector<int> n_pts_list; //lista de numero de pontos por pacote

ofstream myfile;

void ResolveDir();

typedef sick_ldmrs_msgs::SICK_LDMRS_Point PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
ros::Publisher pub_;
ros::Publisher pub_npoints;

// tf2_ros::Buffer *tfBuffer;
// tf2_ros::TransformListener *tfListener;
tf::TransformListener* tf_;

std::string fixed_frame_;
std::string source_frame_;
PointCloudT::Ptr cloud_lower_ = pcl::make_shared<PointCloudT>();

void savePoints(const PointCloudT::ConstPtr& pc){
 ros::Time time = ros::Time::now();

 std::time_t std_time;
 std_time = time.sec;
 char* time_date = std::ctime(&std_time);

 ResolveDir(); //cria ou verifica existencia do diretorio
char* home_path = getenv("HOME");
std::string str_path = std::string(home_path) + "/Pontos/pontos_";
//std::string str_time = std::string(time_date);
std::string str_time = std::to_string(time.sec);
std::string str = str_path + str_time + ".txt";
std::string str_pcd = str_path + str_time + ".pcd";

myfile.open(str);
//ROS_INFO("file n = %s",str.c_str());
int j = 0;
int i_up_to = n_pts_list[j];
for (size_t i = 0; i < pc->points.size(); i++) {
  if (i < i_up_to)
       myfile << pc->points[i].x << " " << pc->points[i].y << " " <<pc->points[i].z << " " << j << endl;
       else{
       j++;
       i_up_to += n_pts_list[j];
     }
  }
myfile.close();
if(pc->points.size() > 0)
	pcl::io::savePCDFileBinary(str_pcd, *pc); //Mais Eficiente ?

ROS_INFO("%lu Pontos salvos em %s !\n",pc->points.size(),str.c_str());
n_pts_list.clear(); //verificar se ja foi 'cleared' ? TODO

}

bool StartAggregation = false;


//Callback da nuvem
void callback(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
  static std_msgs::UInt16 npts_msg;

  static PointCloudT::Ptr cloud = pcl::make_shared<PointCloudT>();
  pcl::fromROSMsg(*pc, *cloud);

int current_cloud_size = cloud->size();
npts_msg.data = current_cloud_size;
pub_npoints.publish(npts_msg);
double x0 = cloud->points[0].x;
double y0 = cloud->points[0].y;
double z0 = cloud->points[0].z;

double xn = cloud->points[current_cloud_size-1].x;
double yn = cloud->points[current_cloud_size-1].y;
double zn = cloud->points[current_cloud_size-1].z;

double dx = xn-x0;
double dy = yn-y0;
double dz = zn-z0;

double dist = sqrtf(dx*dx + dy*dy + dz*dz);

PointT min,max;

pcl::getMinMax3D(*cloud,min,max);
double h = abs(max.x - min.x);

// ROS_INFO("min_x = %f , max_x = %f",min.x,max.x);
// ROS_INFO("min_z = %f , max_z = %f",min.z,max.z);




// ROS_INFO("max distance = %f !",dist);
if(!StartAggregation)
return;

  // ----- transform to fixed frame
  try
  {
    //procura transformada mais recente entre os quadros dados
    //geometry_msgs::TransformStamped tf = tfBuffer->lookupTransform(fixed_frame_,source_frame_,ros::Time(0));
    
    
    //nuvem transformada
    PointCloudT::Ptr cloud_fixed = pcl::make_shared<PointCloudT>();
    

    //pcl_ros::transformPointCloud(*cloud,*cloud_fixed,tf.transform);
    //pcl_ros::transformPointCloud(fixed_frame_,*cloud,*cloud_fixed,*tfBuffer);
      
    //*cloud_lower_ += *cloud_fixed;

    if (!pcl_ros::transformPointCloud(fixed_frame_, *cloud, *cloud_fixed, *tf_))
    {
      ROS_WARN("TF exception in transformPointCloud!");
      return ;
    }
  *cloud_lower_ += *cloud_fixed;
  }

    // catch(tf2::TransformException& ex){
    // ROS_WARN("TF Exception %s", ex.what());
    // return;
    // }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("TF Exception %s", ex.what());
    return ;
  }

  // ----- concatenate lower + upper clouds
  
  n_pts_list.push_back(current_cloud_size);

  


  // ----- publish
  sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cloud_lower_, *msg);
  msg->header.stamp = pc->header.stamp;
  msg->header.frame_id = fixed_frame_;
  pub_.publish(msg); //vamos verificar a necessidade de publicar a nuvem inteira

}

//ler inteiro
bool command_parser(gpar_lidar::Command::Request& req, gpar_lidar::Command::Response& res)
{
		res.success = false;

		switch(req.command){
				case 0:
						ROS_WARN("Stop Merging.. \n");
						res.message = "Parar Agregacao \n";
						StartAggregation = false;
						res.success = true;
						break;
				case 1:
						ROS_WARN("Merging clouds.. \n");
						StartAggregation = true;
						res.message = "Agregando Nuvens...";
						res.success = true;
						break;
				case 2:
						 ROS_WARN("Restarting cloud.. \n");
						cloud_lower_->clear();
						res.message = "Nuvens Limpas!";
						res.success = true;
						break;
				case 3:
						ROS_WARN("Salvando Nuvens...");
						savePoints(cloud_lower_); //Pode ser multithread!
						ROS_WARN("Nuvens Salvas!");
						res.message = "Nuvens Salvas!";
						res.success = true;
						break;
				default:
						break;

		}


		return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_controller");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string input_cloud_node;

  if (!private_nh.getParam("fixed_frame", fixed_frame_))
  {
   fixed_frame_ = "map";
   ROS_WARN("Need to set parameter fixed_frame.. set to \"%s\"",fixed_frame_.c_str());
  }

  if(!private_nh.getParam("input_cloud",input_cloud_node))
   {
    ROS_FATAL("Need to set input node to subscribe to!");
    return 1;
   }
  //ex: ldmrs
     if(!private_nh.getParam("source_frame",source_frame_))
   {
    ROS_FATAL("Need to 'source_frame!'   ");
    return 1;
   }


 ros::ServiceServer service = private_nh.advertiseService("command", command_parser);

  /* Novo ros não compila kinect */
  // tfBuffer = new tf2_ros::Buffer;
  // tfListener = new tf2_ros::TransformListener(*tfBuffer);

  // ros::Subscriber cloud_sub = nh.subscribe(input_cloud_node,10,callback);
  // //Verificar como usar message filter com tf2_ros + source/target frame
  
  tf_ = new tf::TransformListener(nh, ros::Duration(3.0));

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub;
  tf::MessageFilter<sensor_msgs::PointCloud2>* tf_filter;

  sub.subscribe(nh, input_cloud_node, 10);
  tf_filter = new tf::MessageFilter<sensor_msgs::PointCloud2>(sub, *tf_, fixed_frame_, 10);
  tf_filter->registerCallback(boost::bind(callback, _1));

  pub_ = nh.advertise<sensor_msgs::PointCloud2>("map_cloud", 10);
  pub_npoints = nh.advertise<std_msgs::UInt16>("laser_detected_points",10);

  ros::spin();


// delete tfBuffer;
// delete tfListener;
/*  OLD CODE */
  delete tf_filter;
  delete tf_;

  return 0;
}


void ResolveDir(){
struct stat statbuff;
bool isDir = 0;
char* home_path = getenv("HOME");
string pontos_path = std::string(home_path) + "/Pontos/";
if(stat(pontos_path.c_str(),&statbuff) == -1) {
	mkdir(pontos_path.c_str(),0755);
	}

}
