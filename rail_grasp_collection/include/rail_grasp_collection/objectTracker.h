#include <ros/ros.h>
#include <rail_grasp_collection/ClosestObject.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <sensor_msgs/point_cloud_conversion.h>

class objectTracker
{
public:
  ros::NodeHandle n;

  rail_manipulation_msgs::SegmentedObjectList objectList;

  //Services
  ros::ServiceServer closestObjectServer;

  //Subscribers
  ros::Subscriber objectSubscriber;

  /**
  * Constructor
  */
  objectTracker();

  void objectCallback(const rail_manipulation_msgs::SegmentedObjectList &objects);

  bool determineClosestObject(rail_grasp_collection::ClosestObject::Request &req, rail_grasp_collection::ClosestObject::Response &res);
};
