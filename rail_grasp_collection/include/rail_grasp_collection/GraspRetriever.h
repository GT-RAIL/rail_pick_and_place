/*!
 * \file GraspRetriever.h
 * \brief The grasp retriever node object.
 *
 * The grasp retriever allows for loading stored grasps from the grasp database training set. An action server is
 * started as the main entry point to grasp retrieval. A latched topic is used to publish the resulting point cloud and
 * pose.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 4, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASP_RETRIEVER_H_
#define RAIL_PICK_AND_PLACE_GRASP_RETRIEVER_H_

// ROS
#include <actionlib/server/simple_action_server.h>
#include <graspdb/graspdb.h>
#include <rail_pick_and_place_msgs/RetrieveGraspDemonstrationAction.h>
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
  virtual ~GraspRetriever();

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
   * \brief Callback for the retrieve grasp action server.
   *
   * The retrieve grasp action will attempt to load a stored grasp demonstration from the grasp database.
   *
   * \param goal The goal object specifying the parameters.
   */
  void retrieveGrasp(const rail_pick_and_place_msgs::RetrieveGraspDemonstrationGoalConstPtr &goal);

  /*! The okay check flag. */
  bool okay_;
  /*! The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The main action server. */
  actionlib::SimpleActionServer<rail_pick_and_place_msgs::RetrieveGraspDemonstrationAction> as_;
  /*! The latched publishers for retrieved data. */
  ros::Publisher point_cloud_pub_, pose_pub_;
};

}
}

#endif