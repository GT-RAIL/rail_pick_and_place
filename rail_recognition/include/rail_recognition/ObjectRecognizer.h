/*!
 * \file ObjectRecognizer.h
 * \brief The object recognizer node object.
 *
 * The object recognizer sets up an action server that allows the recognition of a single segmented object.
 *
 * \author David Kent, WPI - rctoris@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_OBJECT_RECOGNIZER_H_
#define RAIL_PICK_AND_PLACE_OBJECT_RECOGNIZER_H_

// ROS
#include <actionlib/server/simple_action_server.h>
#include <graspdb/graspdb.h>
#include <rail_manipulation_msgs/RecognizeObjectAction.h>
#include <ros/ros.h>

namespace rail
{
namespace pick_and_place
{

/*!
 * \class ObjectRecognizer
 * \brief The object recognizer node object.
 *
 * The object recognizer sets up an action server that allows the recognition of a single segmented object.
 */
class ObjectRecognizer
{
public:
  /*!
   * \brief Creates a new ObjectRecognizer.
   *
   * Creates a new ObjectRecognizer with the associated action server.
   */
  ObjectRecognizer();

  /*!
   * \brief Cleans up a ObjectRecognizer.
   *
   * Cleans up any connections used by the ObjectRecognizer.
   */
  virtual ~ObjectRecognizer();

  /*!
   * \brief A check for a valid ObjectRecognizer.
   *
   * This function will return true if the appropriate connections were created successfully during initialization.
   *
   * \return True if the appropriate connections were created successfully during initialization.
   */
  bool okay() const;

private:
  /*!
   * \brief The recognize object action server callback.
   *
   * Attempts to recognize the object given in the goal and return it in the result.
   *
   * \param goal The goal specifying the segmented object to recognize.
   */
  void recognizeObjectCallback(const rail_manipulation_msgs::RecognizeObjectGoalConstPtr &goal);

  /*! The okay check flag. */
  bool okay_;
  /*! The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The main recognition action server. */
  actionlib::SimpleActionServer<rail_manipulation_msgs::RecognizeObjectAction> as_;
};

}
}

#endif
