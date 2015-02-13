#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <rail_recognition/ReadGrasp.h>
#include <ros/package.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

class GraspReader
{
public:
  //ROS publishers, subscribers, and action servers
  ros::NodeHandle n;
  
  ros::Publisher cloudPublisher;
  ros::Publisher graspsPublisher;
  std::vector<ros::Publisher> individualGraspPublishers;
  
  ros::ServiceServer readGraspServer;
  
  tf::TransformBroadcaster tfBroadcaster;
  tf::TransformListener tfListener;

  std::string modelDirectory;
  
  /**
   * Constructor
   */
  GraspReader();
  
  /**
   * Service callback for grasp reading
   * @param req service request
   * @param res service response
   * @return true on success
   */
  bool readGraspService(rail_recognition::ReadGrasp::Request &req, rail_recognition::ReadGrasp::Response &res);
};
