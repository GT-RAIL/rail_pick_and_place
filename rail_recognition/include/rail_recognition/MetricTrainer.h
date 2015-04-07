#ifndef RAIL_PICK_AND_PLACE_MODEL_GENERATOR_H_
#define RAIL_PICK_AND_PLACE_MODEL_GENERATOR_H_

// RAIL Recognition
#include "PCLGraspModel.h"

// ROS
#include <geometry_msgs/PoseArray.h>
#include <graspdb/graspdb.h>
#include <pcl_ros/point_cloud.h>
#include <rail_pick_and_place_msgs/TrainMetrics.h>
#include <ros/ros.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// CPP
#include <iostream>
#include <fstream>

namespace rail
{
namespace pick_and_place
{

class MetricTrainer
{
public:
  /*! If a topic should be created to display debug information such as model point clouds. */
  static const bool DEFAULT_DEBUG = false;

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
  bool trainMetrics(rail_pick_and_place_msgs::TrainMetrics::Request &req, rail_pick_and_place_msgs::TrainMetrics::Response &res);

  void generateAndStoreModels(std::vector<PCLGraspModel> &grasp_models, const int max_model_size,
      std::vector<uint32_t> &new_model_ids) const;

  bool registrationCheck(const PCLGraspModel &base, const PCLGraspModel &target, PCLGraspModel &result) const;

  /*! The debug flag. */
  bool debug_, okay_;
  /*! The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The debug topic publisher. */
  ros::Publisher base_pc_pub_, aligned_pc_pub_;
  ros::ServiceServer train_metric_server_;
};

}
}

#endif
