/*!
 * \file GraspModel.h
 * \brief Trained grasp model information.
 *
 * A grasp model contains a 3D point cloud model made up of several segmented point cloud segments from various grasp
 * demonstrations, an object name (may be non-unique), an array of grasps for the model, and a unique identifier.
 * This class is useful for internal data management within the graspdb library. Convenience functions are added for
 * use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 12, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASPDB_GRASP_MODEL_H_
#define RAIL_PICK_AND_PLACE_GRASPDB_GRASP_MODEL_H_

// graspdb
#include "Grasp.h"

// ROS
#include <rail_pick_and_place_msgs/GraspModel.h>
#include <sensor_msgs/PointCloud2.h>

// C++ Standard Library
#include <string>
#include <vector>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

/*!
 * \class GraspModel
 * \brief Trained grasp model information.
 *
 * A grasp model contains a 3D point cloud model made up of several segmented point cloud segments from various grasp
 * demonstrations, an object name (may be non-unique), an array of grasps for the model, and a unique identifier.
 * This class is useful for internal data management within the graspdb library. Convenience functions are added for
 * use with ROS messages.
 */
class GraspModel : public Entity
{
public:
  /*!
   * \brief Create a new GraspModel.
   *
   * Creates a new GraspModel with the given values. This constructor assumes a valid ID and timestamp are known.
   *
   * \param id The unique ID of the database entry (defaults to 0).
   * \param object_name The name of the object grasped (defaults to the empty string).
   * \param grasps The pose of the grasp (defaults to an empty vector).
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object (defaults to empty values).
   * \param created The created timestamp (defaults to 0).
   */
  GraspModel(const uint32_t id = Entity::UNSET_ID, const std::string &object_name = "",
      const std::vector<Grasp> &grasps = std::vector<Grasp>(),
      const sensor_msgs::PointCloud2 &point_cloud = sensor_msgs::PointCloud2(),
      const time_t created = Entity::UNSET_TIME);

  /*!
   * \brief Create a new GraspModel.
   *
   * Creates a new GraspModel with the given values. This constructor assumes no valid ID and timestamp are known (e.g.,
   * for use when inserting into the database).
   *
   * \param object_name The name of the object grasped.
   * \param grasps The pose of the grasp.
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object.
   */
  GraspModel(const std::string &object_name, const std::vector<Grasp> &grasps,
      const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Create a new GraspModel.
   *
   * Creates a new GraspModel with the given values the ROS message.
   *
   * \param m The ROS GraspModel message to extract values from.
   */
  GraspModel(const rail_pick_and_place_msgs::GraspModel &gm);

  /*!
   * \brief Object name value accessor.
   *
   * Get the object name value of this GraspModel.
   *
   * \return The object name value.
   */
  const std::string &getObjectName() const;

  /*!
   * \brief Object name value mutator.
   *
   * Set the object name value of this GraspModel.
   *
   * \param object_name The new object name value.
   */
  void setObjectName(const std::string &object_name);

  /*!
   * \brief Grasps value accessor (immutable).
   *
   * Get the grasps of this GraspModel.
   *
   * \return The grasps.
   */
  const std::vector<Grasp> &getGrasps() const;

  /*!
   * \brief Grasps value accessor.
   *
   * Get the grasps of this GraspModel.
   *
   * \return The grasps.
   */
  std::vector<Grasp> &getGrasps();

  /*!
   * \brief Grasps size accessor.
   *
   * Get the number of grasps of this GraspModel.
   *
   * \return The number of grasps of this GraspModel.
   */
  size_t getNumGrasps() const;

  /*!
   * \brief Grasp pose value accessor (immutable).
   *
   * Get the grasp of this GraspModel at the given index.
   *
   * \param i The index of the Pose to get.
   * \return The grasp pose at the given index.
   * \throws std::out_of_range Thrown if the grasp at the given index does not exist.
   */
  const Grasp &getGrasp(const size_t index) const;

  /*!
   * \brief Grasp pose value accessor.
   *
   * Get the grasp of this GraspModel at the given index.
   *
   * \param i The index of the Pose to get.
   * \return The grasp pose at the given index.
   * \throws std::out_of_range Thrown if the grasp at the given index does not exist.
   */
  Grasp &getGrasp(const size_t index);

  /*!
   * \brief Grasp adder.
   *
   * Add the grasp to this GraspModel. If the grasp_model_id of the grasp does not match, it will not be added.
   *
   * \param grasp The new grasp to add.
   */
  void addGrasp(const Grasp &grasp);

  /*!
   * \brief Grasp remover.
   *
   * Remove the grasp at the given index. An invalid index results in no effect.
   *
   * \param i The index of the grasp pose to remove.
   * \throws std::out_of_range Thrown if the grasp at the given index does not exist.
   */
  void removeGrasp(const size_t index);

  /*!
   * \brief Get the best grasp.
   *
   * Get the best grasp based on the highest success rate. A tie results in the smaller index.
   *
   * \return The grasp with the highest success rate.
   * \throws std::out_of_range Thrown if there are no grasps in this grasp model.
   */
  const Grasp &getBestGrasp() const;

  /*!
   * \brief Get the best grasp (immutable).
   *
   * Get the best grasp based on the highest success rate. A tie results in the smaller index.
   *
   * \return The grasp with the highest success rate.
   * \throws std::out_of_range Thrown if there are no grasps in this grasp model.
   */
  Grasp &getBestGrasp();

  /*!
   * \brief Get the best grasp index.
   *
   * Get the index of the best grasp based on the highest success rate. A tie results in the smaller index.
   *
   * \return The index of the grasp with the highest success rate.
   * \throws std::out_of_range Thrown if there are no grasps in this grasp model.
   */
  size_t getBestGraspIndex() const;

  /*!
   * \brief Get the highest success rate.
   *
   * Get the highest grasp success rate of this model.
   *
   * \return The the highest success rate of all grasps for this model.
   * \throws std::out_of_range Thrown if there are no grasps in this grasp model.
   */
  double getBestSuccessRate() const;

  /*!
   * \brief Get the worst grasp (immutable).
   *
   * Get the worst grasp based on the lowest success rate. A tie results in the smaller index.
   *
   * \return The grasp with the lowest success rate.
   * \throws std::out_of_range Thrown if there are no grasps in this grasp model.
   */
  const Grasp &getWorstGrasp() const;

  /*!
   * \brief Get the worst grasp.
   *
   * Get the worst grasp based on the lowest success rate. A tie results in the smaller index.
   *
   * \return The grasp with the lowest success rate.
   * \throws std::out_of_range Thrown if there are no grasps in this grasp model.
   */
  Grasp &getWorstGrasp();

  /*!
   * \brief Get the worst grasp index.
   *
   * Get the index of the worst grasp based on the lowest success rate. A tie results in the smaller index.
   *
   * \return The index of the grasp with the lowest success rate.
   * \throws std::out_of_range Thrown if there are no grasps in this grasp model.
   */
  size_t getWorstGraspIndex() const;

  /*!
   * \brief Get the lowest success rate.
   *
   * Get the lowest grasp success rate of this model.
   *
   * \return The the lowest success rate of all grasps for this model.
   * \throws std::out_of_range Thrown if there are no grasps in this grasp model.
   */
  double getWorstSuccessRate() const;

  /*!
   * \brief Point cloud accessor (immutable).
   *
   * Get the point cloud message.
   *
   * \return The point cloud message.
   */
  const sensor_msgs::PointCloud2 &getPointCloud() const;

  /*!
   * \brief Point cloud accessor.
   *
   * Get the point cloud message.
   *
   * \return The point cloud message.
   */
  sensor_msgs::PointCloud2 &getPointCloud();

  /*!
   * \brief Point cloud mutator.
   *
   * Set the point cloud message to the given values based on the ROS message.
   *
   * \param point_cloud The ROS PointCloud2 message to store.
   */
  void setPointCloud(const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * Converts this GraspModel object into a ROS GraspModel message.
   *
   * \return The ROS GraspModel message with this grasp model data.
   */
  rail_pick_and_place_msgs::GraspModel toROSGraspModelMessage() const;

private:
  /*! The name of the object for this model. */
  std::string object_name_;
  /*! The grasps for this model. */
  std::vector<Grasp> grasps_;
  /*! The point cloud data. */
  sensor_msgs::PointCloud2 point_cloud_;
};

}
}
}

#endif
