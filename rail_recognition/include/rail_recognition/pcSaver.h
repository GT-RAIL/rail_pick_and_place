#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <rail_segmentation/Segment.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>

#include <boost/thread/thread.hpp>
#include <stdlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class pcSaver
{
public:
  int count;

  //ROS publishers, subscribers, and action servers
  ros::NodeHandle n;

  ros::Publisher cloudPublisher;

  ros::ServiceClient segmentationClient;
  ros::ServiceServer segmentationServer;

  //Point clouds
  std::vector<sensor_msgs::PointCloud2> clouds;

  /**
  * Constructor
  */
  pcSaver();

  /**
  * Callback for the point cloud segmentation
  * @param req service request to segment point clouds and store them as pcd files
  * @param res service response after segmentation including segmented point clouds and a boolean value for success
  */
  bool segmentService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
  * Point cloud publishing for ROS visualization
  */
  void publishClouds();
};
