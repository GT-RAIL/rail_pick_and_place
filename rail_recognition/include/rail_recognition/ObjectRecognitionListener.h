#ifndef OBJECT_RECOGNITION_LISTENER_H_
#define OBJECT_RECOGNITION_LISTENER_H_

//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread/recursive_mutex.hpp>
#include <graspdb/graspdb.h>
#include <rail_recognition/Model.h>
#include <rail_recognition/PCRecognizer.h>
#include <rail_manipulation_msgs/RecognizeAction.h>
#include <rail_manipulation_msgs/RecognizeAllAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//C++
#include <boost/thread/thread.hpp>
#include <stdlib.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

//Point Cloud Filtering Constants
#define RADIUS .01
#define NUM_NEIGHBORS 23
#define DST_THRESHOLD .00075

//Recognition Constants
#define ALPHA .5

namespace rail
{
namespace pick_and_place
{

class ObjectRecognitionListener
{
public:

/**
  * Constructor
  */
  ObjectRecognitionListener();

private:
  //ROS publishers, subscribers, and action servers
  ros::NodeHandle n;

  ros::Subscriber segmentedObjectsSubscriber;
  ros::Publisher recognizedObjectsPublisher;

  actionlib::SimpleActionServer<rail_manipulation_msgs::RecognizeAction> asRecognize;
  actionlib::SimpleActionServer<rail_manipulation_msgs::RecognizeAllAction> asRecognizeAll;

  graspdb::Client *graspdb;
  PCRecognizer recognizer;
  rail_manipulation_msgs::SegmentedObjectList objectList;

  float xTrans;
  float yTrans;
  float zTrans;

  boost::recursive_mutex api_mutex;

  //tf
  tf::TransformListener tfListener;
  tf::TransformBroadcaster tfBroadcaster;

  void objectsCallback(const rail_manipulation_msgs::SegmentedObjectList& msg);

  void executeRecognize(const rail_manipulation_msgs::RecognizeGoalConstPtr &goal);

  void executeRecognizeAll(const rail_manipulation_msgs::RecognizeAllGoalConstPtr &goal);
};

} //end namespace pick_and_place
} //end namespace rail

#endif
