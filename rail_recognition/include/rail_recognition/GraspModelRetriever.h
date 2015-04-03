/*!
 * \file GraspModelRetriever.h
 * \brief The grasp model retriever node object.
 *
 * The grasp model retriever allows for loading stored models from the grasp database training set. An action server is
 * started as the main entry point to grasp retrieval. A latched topic is used to publish the resulting point cloud and
 * pose array.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 3, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASP_MODEL_RETRIEVER_H_
#define RAIL_PICK_AND_PLACE_GRASP_MODEL_RETRIEVER_H_

// ROS
#include <actionlib/server/simple_action_server.h>
#include <graspdb/graspdb.h>
#include <rail_pick_and_place_msgs/RetrieveGraspModelAction.h>
#include <ros/ros.h>

namespace rail
{
namespace pick_and_place
{

/*!
 * \class GraspModelRetriever
 * \brief The grasp model retriever node object.
 *
 * The grasp model retriever allows for loading stored models from the grasp database training set. An action server is
 * started as the main entry point to grasp retrieval. A latched topic is used to publish the resulting point cloud and
 * pose array.
 */
class GraspModelRetriever
{
public:
  /*!
   * \brief Create a GraspModelRetriever and associated ROS information.
   *
   * Creates a ROS node handle, creates a client to the grasp database, and starts the action server.
   */
  GraspModelRetriever();

  /*!
   * \brief Cleans up a GraspModelRetriever.
   *
   * Cleans up any connections used by the GraspModelRetriever.
   */
  virtual ~GraspModelRetriever();

  /*!
   * \brief A check for a valid GraspModelRetriever.
   *
   * This function will return true if the appropriate connections were created successfully during initialization.
   *
   * \return True if the appropriate connections were created successfully during initialization.
   */
  bool okay() const;

private:
  /*!
   * \brief Callback for the retrieve grasp model action server.
   *
   * The retrieve grasp model action will attempt to load a stored grasp model from the grasp database.
   *
   * \param goal The goal object specifying the parameters.
   */
  void retrieveGraspModel(const rail_pick_and_place_msgs::RetrieveGraspModelGoalConstPtr &goal);

  /*! The okay check flag. */
  bool okay_;
  /*! The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The main action server. */
  actionlib::SimpleActionServer<rail_pick_and_place_msgs::RetrieveGraspModelAction> as_;
  /*! The latched publishers for retrieved data. */
  ros::Publisher point_cloud_pub_, poses_pub_;
};

}
}

#endif