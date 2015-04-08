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

class MetricTrainer
{
public:
  MetricTrainer();

  /*!
   * \brief Cleans up a ModelGenerator.
   *
   * Cleans up any connections used by the ModelGenerator.
   */
  virtual ~MetricTrainer();

  /*!
   * \brief A check for a valid ModelGenerator.
   *
   * This function will return true if the appropriate connections were created successfully during initialization.
   *
   * \return True if the appropriate connections were created successfully during initialization.
   */
  bool okay() const;

private:
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
