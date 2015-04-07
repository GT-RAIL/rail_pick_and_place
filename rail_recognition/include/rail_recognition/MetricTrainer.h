#ifndef RAIL_PICK_AND_PLACE_METRIC_TRAINER_H_
#define RAIL_PICK_AND_PLACE_METRIC_TRAINER_H_

// ROS
#include <graspdb/graspdb.h>
#include <rail_pick_and_place_msgs/TrainMetrics.h>
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
  bool trainMetrics(rail_pick_and_place_msgs::TrainMetrics::Request &req,
      rail_pick_and_place_msgs::TrainMetrics::Response &res);

  /*! The okay check flag. */
  bool okay_;
  /*! The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The debug topic publisher. */
  ros::Publisher base_pc_pub_, aligned_pc_pub_;
  ros::ServiceServer train_metrics_srv_;
};

}
}

#endif
