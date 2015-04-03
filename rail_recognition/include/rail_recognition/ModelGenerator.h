// ROS
#include <actionlib/server/simple_action_server.h>
#include <graspdb/graspdb.h>
#include <rail_pick_and_place_msgs/GenerateModelsAction.h>
#include <ros/ros.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// RAIL Recognition
#include "PCLGraspModel.h"

namespace rail
{
namespace pick_and_place
{

class ModelGenerator
{
public:
  /*! If a topic should be created to display debug information such as model point clouds. */
  static const bool DEFAULT_DEBUG = false;

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
  void generateModelsCallback(const rail_pick_and_place_msgs::GenerateModelsGoalConstPtr &goal);

  void generateAndStoreModels(std::vector<PCLGraspModel> &grasp_models, const int max_model_size,
      std::vector<uint32_t> &new_model_ids) const;

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
