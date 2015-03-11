/*!
 * \file Model.h
 * \brief Trained grasp model information.
 *
 * A grasp model contains a 3D point cloud model made up of several segmented point cloud segments from various grasp
 * demonstrations, an object name (may be non-unique), an array of grasp poses for the model, a unique identifier,
 * counters for trials/success rates for the grasps, and a timestamp.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

#ifndef RAIL_GRASPDB_MODEL_H_
#define RAIL_GRASPDB_MODEL_H_

#include <graspdb/Pose.h>
#include <rail_pick_and_place_msgs/Model.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

/*!
 * \class Model
 * \brief Trained grasp model information.
 *
 * A grasp model contains a 3D point cloud model made up of several segmented point cloud segments from various grasp
 * demonstrations, an object name (may be non-unique), an array of grasp poses for the model, a unique identifier,
 * counters for trials/success rates for the grasps, and a timestamp.
 */
class Model
{
public:
  /*! The default value for an unset identifier (i.e., a model not yet in the database). */
  static const uint32_t UNSET_ID = 0;
  /*! The default value for an unset timestamp (i.e., a model not yet in the database). */
  static const time_t UNSET_TIME = 0;

  /*!
   * \brief Create a new Model.
   *
   * Creates a new Model with the given values. This constructor assumes a valid ID and timestamp are known. Models
   * are initialized with an empty grasp list and must be added manually to insure consistency with the success rates.
   *
   * \param id The unique ID of the database entry (defaults to 0).
   * \param object_name The name of the object grasped (defaults to the empty string).
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object (defaults to empty values).
   * \param created The created timestamp (defaults to 0).
   */
  Model(const uint32_t id = UNSET_ID, const std::string &object_name = "",
      const sensor_msgs::PointCloud2 &point_cloud = sensor_msgs::PointCloud2(), const time_t created = UNSET_TIME);

  /*!
   * \brief Create a new Model.
   *
   * Creates a new Model with the given values. This constructor assumes no valid ID and timestamp are known (e.g.,
   * for use when inserting into the database). Models are initialized with an empty grasp list and must be added
   * manually to insure consistency with the success rates.
   *
   * \param object_name The name of the object grasped.
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object (defaults to empty values).
   */
  Model(const std::string &object_name, const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Create a new Model.
   *
   * Creates a new Model with the given values the ROS message.
   *
   * \param m The ROS Model message to extract values from.
   */
  Model(const rail_pick_and_place_msgs::Model &m);

  /*!
   * \brief ID value accessor.
   *
   * Get the ID value of this Model.
   *
   * \return The ID value.
   */
  uint32_t getID() const;

  /*!
   * \brief ID value mutator.
   *
   * Set the ID value of this Model.
   *
   * \param id The new ID value.
   */
  void setID(const uint32_t id);

  /*!
   * \brief Object name value accessor.
   *
   * Get the object name value of this Model.
   *
   * \return The object name value.
   */
  const std::string &getObjectName() const;

  /*!
   * \brief Object name value mutator.
   *
   * Set the object name value of this Model.
   *
   * \param object_name The new object name value.
   */
  void setObjectName(const std::string &object_name);

  /*!
   * \brief Grasp poses value accessor.
   *
   * Get the grasp poses of this Model.
   *
   * \return The grasp poses.
   */
  const std::vector<Pose> &getGraspPoses() const;

  /*!
   * \brief Grasp poses size accessor.
   *
   * Get the number of grasp poses of this Model.
   *
   * \return The number of grasp poses of this Model.
   */
  size_t getNumGraspPoses() const;

  /*!
   * \brief Grasp pose value accessor.
   *
   * Get the grasp pose of this Model at the given index.
   *
   * \param i The index of the Pose to get.
   * \return The grasp pose at the given index.
   */
  const Pose &getGraspPose(const size_t i) const;

  /*!
   * \brief Grasp pose adder.
   *
   * Add the grasp pose of this Model. Success rates are set to a default of 0.
   *
   * \param grasp_poses The new grasp pose to add.
   * \param grasp_frame_id The frame ID this grasp is in reference to.
   */
  void addGraspPose(const Pose &grasp_pose, const std::string &grasp_frame_id);

  /*!
   * \brief Grasp pose adder.
   *
   * Add the grasp pose of this Model with the given success rates and attempts.
   *
   * \param grasp_poses The new grasp pose to add.
   * \param grasp_frame_id The frame ID this grasp is in reference to.
   * \param successes The associated success rates.
   * \param attempts The associated attempts.
   */
  void addGraspPose(const Pose &grasp_pose, const std::string &grasp_frame_id, const uint32_t successes,
      const uint32_t attempts);

  /*!
   * \brief Grasp pose remover.
   *
   * Remove the grasp pose and associated success rates and attempts at the given index. An invalid index results in
   * no effect.
   *
   * \param i The index of the grasp pose to remove.
   */
  void removeGraspPose(const size_t i);

  /*!
   * \brief Success values accessor.
   *
   * Get the number of successful grasps for all grasps.
   *
   * \return The number of successful grasps for each grasp.
   */
  const std::vector<uint32_t> &getSuccesses() const;

  /*!
   * \brief Grasp frame ID value accessor.
   *
   * Get the grasp frame ID for the grasp at the given index.
   *
   * \param i The index of the grasp to get the grasp frame ID for.
   * \return The grasp frame ID for the grasp at the given index.
   */
  const std::string &getGraspFrameID(size_t i) const;

  /*!
   * \brief Grasp frame IDs accessor.
   *
   * Get the grasp frame IDs for all grasps.
   *
   * \return The grasp frame IDs for each grasp.
   */
  const std::vector<std::string> &getGraspFrameIDs() const;

  /*!
   * \brief Success value accessor.
   *
   * Get the number of successful grasps for the grasp at the given index. An invalid index will result in 0.
   *
   * \param i The index of the grasp to get the number of successful grasps for.
   * \return The number of successful grasps for the grasp at the given index.
   */
  uint32_t getSuccesses(size_t i) const;

  /*!
   * \brief Attempt values accessor.
   *
   * Get the number of attempted grasps for all grasps.
   *
   * \return The number of attempted grasps for each grasp.
   */
  const std::vector<uint32_t> &getAttempts() const;

  /*!
   * \brief Attempt value accessor.
   *
   * Get the number of attempted grasps for the grasp at the given index. An invalid index will result in 0.
   *
   * \param i The index of the grasp to get the number of attempted grasps for.
   * \return The number of attempted grasps for the grasp at the given index.
   */
  uint32_t getAttempts(size_t i) const;

  /*!
   * \brief Success rates accessor.
   *
   * Get the success rates of the grasps. This will create an return a new vector.
   *
   * \return The success rates for each grasp.
   */
  std::vector<double> getSuccessRates() const;

  /*!
   * \brief Success rate accessor.
   *
   * Get the success rate of the grasp at the given index. An invalid index will return -1. A 0 attempt grasp results
   * in a 0 value.
   *
   * \param i The index of the grasp to get the success rate for.
   * \return The success rate for the grasp at the given index.
   */
  double getSuccessRate(size_t i) const;

  /*!
   * \brief Get the best grasp index.
   *
   * Get the index of the best grasp based on the highest success rate. A tie results in the smaller index. A empty
   * list will result in a return value of 0.
   *
   * \return The index of the grasp with the highest success rate.
   */
  size_t getBestGraspIndex() const;

  /*!
   * \brief Point cloud accessor.
   *
   * Get the point cloud message.
   *
   * \return The point cloud message.
   */
  const sensor_msgs::PointCloud2 &getPointCloud() const;

  /*!
   * \brief Point cloud buffer mutator.
   *
   * Set the point cloud message to the given values based on the ROS message.
   *
   * \param point_cloud The ROS PointCloud2 message to store.
   */
  void setPointCloud(const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Created timestamp value accessor.
   *
   * Get the created timestamp value of this Model.
   *
   * \return The created timestamp value.
   */
  time_t getCreated() const;

  /*!
   * \brief Created timestamp value mutator.
   *
   * Set the created timestamp value of this Model.
   *
   * \param created The new created timestamp value.
   */
  void setCreated(const time_t created);

  /*!
   * Converts this Model object into a ROS Model message.
   *
   * \return The ROS Model message with this model data.
   */
  rail_pick_and_place_msgs::Model toROSModelMessage() const;

private:
  /*! The ID. */
  uint32_t id_;
  /*! The name of the object for this model. */
  std::string object_name_;
  /*! The point cloud data. */
  sensor_msgs::PointCloud2 point_cloud_;
  /*! The grasp pose data. */
  std::vector<Pose> grasp_poses_;
  /*! The grasp pose frame data. */
  std::vector<std::string> grasp_frame_ids_;
  /*! The grasp success data. */
  std::vector<uint32_t> successes_;
  /*! The grasp attempt data. */
  std::vector<uint32_t> attempts_;
  /*! The created timestamp. */
  time_t created_;
};

}
}
}

#endif
