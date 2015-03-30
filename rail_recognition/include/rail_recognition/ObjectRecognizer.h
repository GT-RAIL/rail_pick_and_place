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

class ObjectRecognizer
{
public:
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
  void recognizeObjectCallback(const rail_manipulation_msgs::RecognizeObjectGoalConstPtr &goal);

  /*! The okay check flag. */
  bool okay_;
  /* The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The main recognition action server. */
  actionlib::SimpleActionServer<rail_manipulation_msgs::RecognizeObjectAction> as_;
};

}
}

#endif
