#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <std_srvs/Trigger.h>

tf::TransformListener *tf_;
bool has_reference_param = false;
std::string reference;

std::string save_path;

bool is_ply = false;

static sensor_msgs::PointCloud2::Ptr cloud_g = boost::make_shared<sensor_msgs::PointCloud2>();
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

bool got_cloud = false;
void cloud_callback(const sensor_msgs::PointCloud2::Ptr cloud)
{
  ROS_INFO("GOT CLOUD!");

  if (got_cloud == false)
  {
    ROS_INFO("N Points: %d", cloud->height * cloud->width);
    ROS_INFO("N Fields %ld", cloud->fields.size());

    got_cloud = true;
  }

  cloud_g = cloud;
}

bool save_map(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
  static pcl::PCLPointCloud2::Ptr pcl_cloud = pcl::make_shared<pcl::PCLPointCloud2>();
  // pcl::io::savePCDFile("teste.pcd",*cloud2);

  std::string ref_;
  if (has_reference_param)
  {
    ref_ = reference;
  }
  else
  {
    ref_ = cloud_g->header.frame_id;
  }

  try
  {
    if (!pcl_ros::transformPointCloud(ref_, *cloud_g, *cloud_g, *tf_))
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
  pcl_conversions::moveToPCL(*cloud_g, *pcl_cloud);
  std::time_t std_time;
  char *time_date = std::ctime(&std_time);
  ros::Time time = ros::Time::now();
  std_time = time.sec;
  std::string str_time = std::to_string(time.sec);
  std::string str_filename = save_path + "/points_sweep_" + str_time;
  ;
  // pcl::io::savePCDFileBinary(str_pcd,*cloud);

  // TODO Verificar exceptions
  if (!is_ply)
  {
    str_filename += ".pcd";
    pcl::io::savePCDFile(str_filename, *pcl_cloud); // May be binary
    ROS_INFO("PCD Saved at %s", str_filename.c_str());
  }
  else
  {
    str_filename += ".ply";
    pcl::io::savePLYFile(str_filename, *pcl_cloud);
    ROS_INFO("PLY Saved at %s", str_filename.c_str());
  }
  resp.success = true;
  resp.message = "saved at " + str_filename;
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

  std::string file_format;

  nh.param<std::string>("file_format", file_format, "pcd");

  if (file_format == "pcd")
  {
    is_ply = false;
  }
  else if (file_format == "ply")
  {
    is_ply = true;
  }
  else
  {
    ROS_ERROR("File Format not supported.");
    ros::shutdown();
    exit(-1);
  }

  ROS_WARN("File format: %s", file_format.c_str());

  if (!nh.getParam("reference", reference))
  {
    has_reference_param = false;
    ROS_WARN("Using laser frame of reference");
  }
  else
  {
    ROS_WARN("Using reference -> %s", reference.c_str());
  }

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

  ROS_WARN("subscribed to cloud topic %s. use service ~/save_cloud to save the point cloud", cloud_topic.c_str());
  ROS_WARN("Clouds are saved at %s", save_path.c_str());

  tf_ = new tf::TransformListener(nh, ros::Duration(3.0));

  ros::spin();

  return 0;
}
