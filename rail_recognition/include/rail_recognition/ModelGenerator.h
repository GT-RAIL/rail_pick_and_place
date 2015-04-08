/*!
 * \file ModelGenerator.h
 * \brief The grasp model generator node object.
 *
 * The grasp model generator allows for generating graspdb models based on registration criteria. An action server is
 * used to provide the model/grasp demonstration IDs to use during registration.
 *
 * \author David Kent, WPI - rctoris@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 6, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_MODEL_GENERATOR_H_
#define RAIL_PICK_AND_PLACE_MODEL_GENERATOR_H_

// RAIL Recognition
#include "PCLGraspModel.h"

// ROS
#include <actionlib/server/simple_action_server.h>
#include <graspdb/graspdb.h>
#include <rail_pick_and_place_msgs/GenerateModelsAction.h>
#include <ros/ros.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rail
{
namespace pick_and_place
{

/*!
 * \class ModelGenerator
 * \brief The grasp model generator node object.
 *
 * The grasp model generator allows for generating graspdb models based on registration criteria. An action server is
 * used to provide the model/grasp demonstration IDs to use during registration.
 */
class ModelGenerator
{
public:
  /*! If a topic should be created to display debug information such as model point clouds. */
  static const bool DEFAULT_DEBUG = false;

  /*!
   * \brief Create a ModelGenerator and associated ROS information.
   *
   * Creates a ROS node handle and starts the action server.
   */
  ModelGenerator();

  /*!
   * \brief Cleans up a ModelGenerator.
   *
   * Cleans up any connections used by the ModelGenerator.
   */
  virtual ~ModelGenerator();

  /*!
   * \brief A check for a valid ModelGenerator.
   *
   * This function will return true if the appropriate connections were created successfully during initialization.
   *
   * \return True if the appropriate connections were created successfully during initialization.
   */
  bool okay() const;

private:
  /*!
   * \brief Main model generation callback.
   *
   * The main callback for the model generation action server.
   *
   * \param goal The goal specifying the parameters for the model generation.
   */
  void generateModelsCallback(const rail_pick_and_place_msgs::GenerateModelsGoalConstPtr &goal);

  /*!
   * \brief Model generation function.
   *
   * Attempt to search for valid registration pairs for the given models up to the max model size. Valid models are
   * saved to the database and a list of IDs is stored in the given vector.
   *
   * \param grasp_models The array of grasp models to attempt to generate models for.
   * \param max_model_size The maximum number of grasps allowed per model.
   * \param new_model_ids The vector to fill with the new grasp model IDs.
   */
  void generateAndStoreModels(std::vector<PCLGraspModel> &grasp_models, const int max_model_size,
      std::vector<uint32_t> &new_model_ids);

  /*!
   * \brief Check the point cloud registration for the two models.
   *
   * Attempt to merge the two models into a single model. If the model passes the criteria, the result model is
   * filled with the corresponding model.
   *
   * \param base The base model to compare to.
   * \param target The target model to compare against the base model.
   * \param new_model_ids The resuliting model (if valid)
   * \return True if the registration meets the criteria.
   */
  bool registrationCheck(const PCLGraspModel &base, const PCLGraspModel &target, PCLGraspModel &result) const;

  /*! The debug flag. */
  bool debug_, okay_;
  /*! The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The main model generation action server. */
  actionlib::SimpleActionServer<rail_pick_and_place_msgs::GenerateModelsAction> as_;
  /*! The debug topic publisher. */
  ros::Publisher debug_pc_pub_, debug_poses_pub_;
};

}
}

#endif
