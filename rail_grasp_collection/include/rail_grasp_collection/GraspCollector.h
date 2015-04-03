/*!
 * \file GraspCollector.h
 * \brief The main grasp collector node object.
 *
 * The grasp collector is responsible for capturing and storing grasps. An action server is started is the main entry
 * point to grasp collecting.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 3, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASP_COLLECTOR_H_
#define RAIL_PICK_AND_PLACE_GRASP_COLLECTOR_H_

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Point32.h>
#include <graspdb/graspdb.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/LiftAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_manipulation_msgs/VerifyGraspAction.h>
#include <rail_pick_and_place_msgs/GraspAndStoreAction.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/transform_listener.h>

// Boost
#include <boost/thread/mutex.hpp>

namespace rail
{
namespace pick_and_place
{

/*!
 * \class GraspCollector
 * \brief The main grasp collector node object.
 *
 * The grasp collector is responsible for capturing and storing grasps. An action server is started is the main entry
 * point to grasp collecting.
 */
class GraspCollector
{
public:
  /*! If a topic should be created to display debug information such as point clouds. */
  static const bool DEFAULT_DEBUG = false;
  /*! The default wait time for action servers in seconds. */
  static const int AC_WAIT_TIME = 10;

  /*!
   * \brief Create a GraspCollector and associated ROS information.
   *
   * Creates a ROS node handle, subscribes to the relevant topics and servers, and creates a client to the grasp
   * database.
   */
  GraspCollector();

  /*!
   * \brief Cleans up a GraspCollector.
   *
   * Cleans up any connections used by the GraspCollector.
   */
  virtual ~GraspCollector();

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
   * The store grasp action will close the gripper, lift the arm (if specified), verify the grasp (if specified),
   * and store the grasp/point cloud data in the grasp database.
   *
   * \param goal The goal object specifying the parameters.
   */
  void graspAndStore(const rail_pick_and_place_msgs::GraspAndStoreGoalConstPtr &goal);

  /*!
   * \brief Callback for the segmented objects topic.
   *
   * Stores the object list internally.
   *
   * \param object_list The object list returned from the segmented objects topic.
   */
  void segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList::Ptr &object_list);

  /*! Mutex for locking on the segmented object list. */
  boost::mutex mutex_;

  /*! The debug and okay check flags. */
  bool debug_, okay_;
  /*! Frame IDs to use. */
  std::string robot_fixed_frame_id_, eef_frame_id_;
  /*! The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The debug topic publisher. */
  ros::Publisher debug_pub_;
  /*! The listener for the segmented objects. */
  ros::Subscriber segmented_objects_sub_;
  /*! The most recent segmented objects. */
  rail_manipulation_msgs::SegmentedObjectList::Ptr object_list_;
  /*! The main grasp collection action server. */
  actionlib::SimpleActionServer<rail_pick_and_place_msgs::GraspAndStoreAction> as_;
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
  /*! The action client timeout. */
  ros::Duration ac_wait_time_;
};

}
}

#endif