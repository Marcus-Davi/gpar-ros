/* Author : Marcus Forte <davi2812@dee.ufc.br> */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <pcl_conversions/pcl_conversions.h>

laser_geometry::LaserProjection projector;
ros::Publisher pub;
std::string cloud_frame_name;

double angle = 0;

// Gets first echo
void laser_callback(const sensor_msgs::MultiEchoLaserScanConstPtr& scan_in){
static sensor_msgs::PointCloud2 cloud;
static sensor_msgs::LaserScan scan_out;

double angle_abs = fabs(angle);

//Filtrar angulo
if(angle != 0){
  double start_angle = scan_in->angle_min;
  double current_angle = scan_in->angle_min;
  scan_out.angle_increment = scan_in->angle_increment;
  scan_out.ranges.resize(scan_in->ranges.size());
  ros::Time start_time = scan_in->header.stamp;
unsigned int count = 0;
//Varre toda a entrada
  for(unsigned int i=0;i < scan_in->ranges.size(); ++i){
    if(start_angle < -angle_abs){
      start_angle += scan_in->angle_increment;
      current_angle += scan_in->angle_increment;
      start_time += ros::Duration(scan_in->time_increment);
    } else {
      scan_out.ranges[count] = scan_in->ranges[count].echoes[0]; //grab first echo only
      
      



    count++;

    if(current_angle + scan_in->angle_increment > angle_abs)
    break;

    current_angle += scan_in->angle_increment;
    }
  }

  scan_out.header.frame_id = scan_in->header.frame_id;
  scan_out.header.stamp = start_time;
  scan_out.angle_min = -angle_abs;
  scan_out.angle_max = angle_abs;
  scan_out.angle_increment = scan_in->angle_increment;
  scan_out.time_increment = scan_in->time_increment;
  scan_out.scan_time = scan_in->scan_time;
  scan_out.range_min = scan_in->range_min;
  scan_out.range_max = scan_in->range_max;
  projector.projectLaser(scan_out,cloud);

} else {
  scan_out.angle_increment = scan_in->angle_increment;
  scan_out.angle_max = scan_in->angle_max;
  scan_out.angle_min = scan_in->angle_min;
  scan_out.header = scan_in->header;
  scan_out.intensities = scan_in->intensities[0].echoes;
  scan_out.range_max = scan_in->range_max;
  scan_out.range_min = scan_in->range_min;
  scan_out.scan_time = scan_in->scan_time;
  scan_out.time_increment = scan_in->time_increment;
  int n = scan_in->ranges.size();
  scan_out.ranges.resize(n);
  for(int i=0;i<n;++i){
    scan_out.ranges[i] = scan_in->ranges[i].echoes[0];  //grab first echo only
  }

  projector.projectLaser(scan_out,cloud);
}




cloud.header.frame_id = cloud_frame_name;
cloud.header.stamp = ros::Time::now();
pub.publish(cloud);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mlasertopc2");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string output_cloud_name = nh.resolveName("scantocloud");
  std::string scan_topic_name = nh.resolveName("scan");

  if(!private_nh.getParam("angle_bound",angle))
   {
     angle = 0;
     ROS_WARN("Need to set parameter 'angle_bound' ");
   }

   if(!private_nh.getParam("cloud_frame",cloud_frame_name)){
    cloud_frame_name = "cloud";
    ROS_WARN("Need to set parameter 'cloud_frame'.. set to \"%s\"",cloud_frame_name.c_str());
   }

   ros::Subscriber sub = nh.subscribe(scan_topic_name,10,laser_callback);
    pub  = nh.advertise<sensor_msgs::PointCloud2>(output_cloud_name, 10);


  ros::spin();



  return 0;
}