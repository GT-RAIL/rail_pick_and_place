#ifndef RAIL_PICK_AND_PLACE_GRASP_COLLECTOR_H_
#define RAIL_PICK_AND_PLACE_GRASP_COLLECTOR_H_

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point32.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/LiftAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_manipulation_msgs/VerifyGraspAction.h>
#include <rail_pick_and_place_msgs/StoreGraspAction.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/transform_listener.h>

namespace rail
{
namespace pick_and_place
{

class GraspCollector
{
public:
  static const bool DEFAULT_DEBUG = false;
  static const int TF_CACHE_TIME = 5;
  static const int AC_WAIT_TIME = 10;

  GraspCollector();

  ~GraspCollector();

private:

  void storeGrasp(const rail_pick_and_place_msgs::StoreGraspGoalConstPtr &goal);

  void segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &object_list);

  int determineClosestObject(const geometry_msgs::Vector3 &v);

  ros::NodeHandle node_, private_node_;

  bool debug_;
  std::string robot_fixed_frame_, grasp_frame_, gripper_action_server_, lift_action_server_, verify_grasp_action_server_;

  ros::Publisher debug_pub_;
  ros::Subscriber segmented_objects_sub_;
  rail_manipulation_msgs::SegmentedObjectList object_list_;
  actionlib::SimpleActionServer<rail_pick_and_place_msgs::StoreGraspAction> as_;
  actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction> *gripper_ac_;
  actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction> *lift_ac_;
  actionlib::SimpleActionClient<rail_manipulation_msgs::VerifyGraspAction> *verify_grasp_ac_;
  std::vector<std::string> finger_frames_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Duration tf_cache_time_, ac_wait_time_;
};

}
}

#endif