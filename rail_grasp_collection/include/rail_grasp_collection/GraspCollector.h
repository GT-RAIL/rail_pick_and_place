/*!
 * \file GraspCollector.h
 * \brief The main grasp collector node object.
 *
 * The grasp collector is responsible for capturing and storing grasps. An action server is started is the main entry point to grasp collecting.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASP_COLLECTOR_H_
#define RAIL_PICK_AND_PLACE_GRASP_COLLECTOR_H_

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point32.h>
#include <graspdb/graspdb.h>
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

/*!
 * \class GraspCollector
 * \brief The main grasp collector node object.
 *
 * The grasp collector is responsible for capturing and storing grasps. An action server is started is the main entry point to grasp collecting.
 */
class GraspCollector
{
public:
  /*! If a topic should be created to display debug information such as point clouds. */
  static const bool DEFAULT_DEBUG = false;
  /*! The default cache time for the TF buffer in seconds. */
  static const int TF_CACHE_TIME = 5;
  /*! The default wait time for action servers in seconds. */
  static const int AC_WAIT_TIME = 10;
  /*! The default PostgreSQL port. */
  static const unsigned int DEFAULT_PORT = 5432;

  /*!
   * \brief Create a GraspCollector and associated ROS information.
   *
   * Creates a ROS node handle, subscribes to the relevant topics and servers, and creates a client to the grasp database.
   */
  GraspCollector();

  /*!
   * \brief Cleans up a GraspCollector.
   *
   * Cleans up any connections used by the GraspCollector.
   */
  ~GraspCollector();

  /*!
   * \brief A check for a valid GraspCollector.
   *
   * This function will return true if the appropriate connections were created successfully during initialization.
   *
   * \return True if the appropriate connections were created successfully during initialization.
   */
  bool okay() const;

private:
  /*!
   * \brief Callback for the store grasp action server.
   *
   * The store grasp action will close the gripper, lift the arm (if specified), verify the grasp (if specified), and store the grasp/point cloud data in the grasp database.
   *
   * \param goal The goal object specifying the parameters.
   */
  void storeGrasp(const rail_pick_and_place_msgs::StoreGraspGoalConstPtr &goal);

  /*!
   * \brief Callback for the segmented objects topic.
   *
   * Stores the object list internally.
   *
   * \param object_list The object list returned from the segmented objects topic.
   */
  void segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &object_list);

  /*!
   * \brief Check for the closest object to the given point.
   *
   * Check for the closest object (cluster of points) to the given point. This will use the most recent list of segmented objects. The index of the closest object is returned or -1 if no clusters have been seen.
   *
   * \param v The vector representing the point to check the distance to.
   * \return The index of the closest object to the given point or -1 if no clusters have been seen.
   */
  int determineClosestObject(const geometry_msgs::Vector3 &v);

  /*! The debug and okay check flags. */
  bool debug_, okay_;
  /*! Various parameters loaded from ROS. */
  std::string robot_fixed_frame_, grasp_frame_, gripper_action_server_, lift_action_server_, verify_grasp_action_server_, host_, user_, password_, db_;
  /* The grasp database connection port. */
  int port_;
  /* The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The debug topic publisher. */
  ros::Publisher debug_pub_;
  /*! The listener for the segmented objects. */
  ros::Subscriber segmented_objects_sub_;
  /*! The most recent segmented objects. */
  rail_manipulation_msgs::SegmentedObjectList object_list_;
  /*! The main grasp collection action server. */
  actionlib::SimpleActionServer<rail_pick_and_place_msgs::StoreGraspAction> as_;
  /*! The gripper action client. */
  actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction> *gripper_ac_;
  /*! The lift action client. */
  actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction> *lift_ac_;
  /*! The verify grasp action client. */
  actionlib::SimpleActionClient<rail_manipulation_msgs::VerifyGraspAction> *verify_grasp_ac_;

  /*! The trasnform tree buffer. */
  tf2_ros::Buffer tf_buffer_;
  /*! The trasnform client. */
  tf2_ros::TransformListener tf_listener_;
  /*! The TF cache time and action client timeout. */
  ros::Duration tf_cache_time_, ac_wait_time_;
};

}
}

#endif