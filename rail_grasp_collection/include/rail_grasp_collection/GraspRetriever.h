/*!
 * \file GraspRetriever.h
 * \brief The grasp retriever node object.
 *
 * The grasp retriever allows for loading stored grasps from the grasp database training set. An action server is
 * started as the main entry point to grasp retrieval.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 4, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASP_RETRIEVER_H_
#define RAIL_PICK_AND_PLACE_GRASP_RETRIEVER_H_

#include <actionlib/server/simple_action_server.h>
#include <graspdb/graspdb.h>
#include <ros/ros.h>

namespace rail
{
namespace pick_and_place
{

/*!
 * \class GraspRetriever
 * \brief The grasp retriever node object.
 *
 * The grasp retriever allows for loading stored grasps from the grasp database training set. An action server is
 * started as the main entry point to grasp retrieval.
 */
class GraspRetriever
{
public:
  /*!
   * \brief Create a GraspRetriever and associated ROS information.
   *
   * Creates a ROS node handle, creates a client to the grasp database, and starts the action server.
   */
  GraspRetriever();

  /*!
   * \brief Cleans up a GraspRetriever.
   *
   * Cleans up any connections used by the GraspRetriever.
   */
  ~GraspCollector();

  /*!
   * \brief A check for a valid GraspRetriever.
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

  /*! The okay check flag. */
  bool debug_, okay_;
  /*! Various parameters loaded from ROS for connection to the grasp database. */
  std::string host_, user_, password_, db_;
  /* The grasp database connection port. */
  int port_;
  /* The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The private ROS node handle. */
  ros::NodeHandle private_node_;
  /*! The main grasp collection action server. */
  actionlib::SimpleActionServer<rail_pick_and_place_msgs::GraspAndStoreAction> as_;
};

}
}

#endif