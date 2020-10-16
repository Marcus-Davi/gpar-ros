#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <gpar_lidar/Commandstr.h>

tf::TransformListener *tf_;
std::string reference = "map";

std::string save_path;

static sensor_msgs::PointCloud2::Ptr cloud_g = boost::make_shared<sensor_msgs::PointCloud2>();
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudT;

void cloud_callback(const sensor_msgs::PointCloud2::Ptr cloud)
{
  ROS_INFO("GOT CLOUD!");
  cloud_g = cloud;
}

bool save_map(gpar_lidar::CommandstrRequest &req, gpar_lidar::CommandstrResponse &resp)
{
  static PointCloudT::Ptr cloud = boost::make_shared<PointCloudT>();
  pcl::fromROSMsg(*cloud_g, *cloud);
  try
  {
	  std::string ref_frame ;
	if(req.command != "") {
		ref_frame = req.command;
	} else {
		ref_frame = reference;
	}

	ROS_WARN("frame = %s",ref_frame.c_str());
    if (!pcl_ros::transformPointCloud(ref_frame, *cloud, *cloud, *tf_))
    {
      ROS_WARN("TF exception in transformPointCloud!");
      resp.message = "Cant transform";
      return false;
    }
  }
  catch (tf::TransformException &ex)
  {
    ROS_WARN("TF Exception %s", ex.what());
    resp.message = "Cant transform";
    return false;
  }
  // Stamp
  std::time_t std_time;
  char *time_date = std::ctime(&std_time);
  ros::Time time = ros::Time::now();
  std_time = time.sec;
  std::string str_time = std::to_string(time.sec);
  std::string str_pcd = save_path + "/points_sweep_" + str_time + ".pcd";
  // pcl::io::savePCDFileBinary(str_pcd,*cloud);

  // TODO Verificar exceptions
  pcl::io::savePCDFileASCII(str_pcd, *cloud); // May be binary
  ROS_INFO("PCD Saved at %s", str_pcd.c_str());
  resp.success = true;
  resp.message = "saved at " + str_pcd;
  return true;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  ros::init(argc, argv, "cloudtofile");

  bool has_color = false;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  std::string cloud_topic = n.resolveName("cloud");

  nh.getParam("reference", reference);

  //set save folder
  char *home_path = getenv("HOME");
  // TODO Criar pasta ?
  std::string save_path_default = std::string(home_path);

  if (!nh.getParam("save_folder", save_path))
  {
    ROS_WARN("to change save folder, set parameter 'save_folder' to a chosen directory");
    save_path = save_path_default;
  }
  {
  }

  ros::ServiceServer save_service = nh.advertiseService("save_cloud", save_map);
  ros::Subscriber sub_cloud = n.subscribe(cloud_topic, 10, cloud_callback);
  ROS_WARN("Saved Cloud Reference -> %s", reference.c_str());
  ROS_WARN("subscribed to cloud topic %s. use service ~/save_cloud to save the point cloud", cloud_topic.c_str());
  ROS_WARN("Clouds are saved at %s", save_path.c_str());

  tf_ = new tf::TransformListener(nh, ros::Duration(3.0));

  ros::spin();

  return 0;
}
