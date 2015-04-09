/*!
 * \file MetricTrainer.h
 * \brief The metric trainer node object.
 *
 * The metric trainer allows for generating data sets for training registration metric decision trees. An action server
 * is used to provide the object name and files are dumped to "registration_metrics.txt".
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_METRIC_TRAINER_H_
#define RAIL_PICK_AND_PLACE_METRIC_TRAINER_H_

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <graspdb/graspdb.h>
#include <rail_pick_and_place_msgs/GetYesNoFeedbackAction.h>
#include <rail_pick_and_place_msgs/TrainMetricsAction.h>
#include <ros/ros.h>

namespace rail
{
namespace pick_and_place
{

/*!
 * \class MetricTrainer
 * \brief The metric trainer node object.
 *
 * The metric trainer allows for generating data sets for training registration metric decision trees. An action server
 * is used to provide the object name and files are dumped to "registration_metrics.txt".
 */
class MetricTrainer
{
public:
  /*!
   * \brief Creates a new MetricTrainer.
   *
   * Creates a new MetricTrainer with the associated topics and action servers.
   */
  MetricTrainer();

  /*!
   * \brief Cleans up a MetricTrainer.
   *
   * Cleans up any connections used by the MetricTrainer.
   */
  virtual ~MetricTrainer();

  /*!
   * \brief A check for a valid MetricTrainer.
   *
   * This function will return true if the appropriate connections were created successfully during initialization.
   *
   * \return True if the appropriate connections were created successfully during initialization.
   */
  bool okay() const;

private:
  /*!
   * \brief The train metrics action server callback.
   *
   * This function starts by loading any grasps with the object name from the goal. All pairs of grasps are joined
   * and published to two different point cloud topics (~/base_pc and ~/aligned_pc). User feedback is requested via
   * an action client call to ~/get_yes_no_feedback. The metrics and the responses are recoreded in the data file.
   *
   * \param goal The goal specifying the object name.
   */
  void trainMetricsCallback(const rail_pick_and_place_msgs::TrainMetricsGoalConstPtr &goal);

  /*! The okay check flag. */
  bool okay_;
  /*! The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The point cloud publishers. */
  ros::Publisher base_pc_pub_, aligned_pc_pub_;
  /*! The train metrics action server. */
  actionlib::SimpleActionServer<rail_pick_and_place_msgs::TrainMetricsAction> as_;
  /*! The user feedback action client. */
  actionlib::SimpleActionClient<rail_pick_and_place_msgs::GetYesNoFeedbackAction> get_yes_and_no_feedback_ac_;
};

}
}

#endif
