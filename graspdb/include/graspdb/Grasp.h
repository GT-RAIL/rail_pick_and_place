/*!
 * \file Grasp.h
 * \brief A grasp database entity.
 *
 * A grasp contains information about a single grasp in the grasp database. This contains information about the grasp
 * pose, end effector frame identifier, number of recorded successful grasps, number of recorded attempted grasps,
 * and associated model ID.  A valid database  entity has an ID and created timestamp. This class is useful for
 * internal data management within the graspdb library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASPDB_GRASP_H_
#define RAIL_PICK_AND_PLACE_GRASPDB_GRASP_H_

// graspdb
#include "Entity.h"
#include "Pose.h"

// ROS
#include <rail_pick_and_place_msgs/GraspWithSuccessRate.h>

// C++ Standard Library
#include <string>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

/*!
 * \class Grasp
 * \brief A grasp database entity.
 *
 * A grasp contains information about a single grasp in the grasp database. This contains information about the grasp
 * pose, end effector frame identifier, number of recorded successful grasps, number of recorded attempted grasps,
 * and associated model ID.  A valid database  entity has an ID and created timestamp. This class is useful for
 * internal data management within the graspdb library. Convenience functions are added for use with ROS messages.
 */
class Grasp : public Entity
{
public:
  /*!
   * \brief Create a new Grasp.
   *
   * Creates a new Grasp with the given values. This constructor assumes a valid ID and timestamp are known.
   *
   * \param id The unique ID of the database entry (defaults to 0).
   * \param grasp_model_id The ID of the associated grasp model entry (defaults to 0).
   * \param grasp_pose The pose of the grasp (defaults to an empty Pose).
   * \param eef_frame_id The frame identifier for the end effector (defaults to the empty string).
   * \param successes The number of recorded successful grasps (defaults to 0).
   * \param attempts The number of recorded attempted grasps (defaults to 0).
   * \param created The created timestamp (defaults to 0).
   */
  Grasp(const uint32_t id = Entity::UNSET_ID, const uint32_t grasp_model_id = Entity::UNSET_ID,
      const Pose &grasp_pose = Pose(), const std::string &eef_frame_id = "", const uint32_t successes = 0,
      const uint32_t attempts = 0, const time_t created = Entity::UNSET_TIME);

  /*!
   * \brief Create a new Grasp.
   *
   * Creates a new Grasp with the given values. This constructor assumes no valid ID and timestamp are known (e.g.,
   * for use when inserting into the database).
   *
   * \param grasp_pose The pose of the grasp.
   * \param grasp_model_id The valid ID of the associated grasp model entry.
   * \param successes The number of recorded successful grasps.
   * \param attempts The number of recorded attempted grasps.
   * \param eef_frame_id The frame identifier for the end effector used (defaults to the empty string).
   */
  Grasp(const Pose &grasp_pose, const uint32_t grasp_model_id, const std::string &eef_frame_id,
      const uint32_t successes, const uint32_t attempts);

  /*!
   * \brief Create a new Grasp.
   *
   * Creates a new Grasp with the given values the ROS message. This constructor assumes no valid ID and timestamp
   * are known.
   *
   * \param gd The ROS grasp message to extract values from.
   * \param grasp_model_id The valid ID of the associated grasp model entry.
   */
  Grasp(const rail_pick_and_place_msgs::GraspWithSuccessRate &g, const uint32_t grasp_model_id);

  /*!
   * \brief Grasp model ID value accessor.
   *
   * Get the grasp model ID value of this Grasp.
   *
   * \return The grasp model ID value.
   */
  uint32_t getGraspModelID() const;

  /*!
   * \brief Grasp model ID value mutator.
   *
   * Set the grasp model ID value of this Grasp.
   *
   * \param grasp_model_id The new grasp model ID value.
   */
  void setGraspModelID(const uint32_t grasp_model_id);

  /*!
   * \brief Grasp pose value accessor (immutable).
   *
   * Get the grasp pose value of this Grasp.
   *
   * \return The grasp pose value.
   */
  const Pose &getGraspPose() const;

  /*!
   * \brief Grasp pose value accessor.
   *
   * Get the grasp pose value of this Grasp.
   *
   * \return The grasp pose value.
   */
  Pose &getGraspPose();

  /*!
   * \brief Grasp pose value mutator.
   *
   * Set the grasp pose value of this Grasp.
   *
   * \param grasp_pose The new grasp pose value.
   */
  void setGraspPose(const Pose &grasp_pose);

  /*!
   * \brief End effector frame ID value accessor.
   *
   * Get the end effector frame ID value of this Grasp.
   *
   * \return The end effector frame ID value.
   */
  const std::string &getEefFrameID() const;

  /*!
   * \brief End effector frame ID value mutator.
   *
   * Set the end effector frame ID value of this Grasp.
   *
   * \param eef_frame_id The new object name value.
   */
  void setEefFrameID(const std::string &eef_frame_id);

  /*!
   * \brief Successes value accessor.
   *
   * Get the successes value of this Grasp.
   *
   * \return The successes value.
   */
  uint32_t getSuccesses() const;

  /*!
   * \brief Successes value mutator.
   *
   * Set the successes value of this Grasp.
   *
   * \param successes The new successes value.
   */
  void setSuccesses(const uint32_t successes);

  /*!
   * \brief Attempts value accessor.
   *
   * Get the attempts value of this Grasp.
   *
   * \return The attempts value.
   */
  uint32_t getAttempts() const;

  /*!
   * \brief Attempts value mutator.
   *
   * Set the attempts value of this Grasp.
   *
   * \param attempts The new attempts value.
   */
  void setAttempts(const uint32_t attempts);

  /*!
   * \brief Get the success rate.
   *
   * Set the success rate of this Grasp. A grasp that has no attempts will return 0.
   *
   * \param attempts The success rate of this Grasp.
   */
  double getSuccessRate() const;

  /*!
   * Converts this Grasp object into a ROS GraspWithSuccessRate message.
   *
   * \return The ROS GraspWithSuccessRate message with this grasp data.
   */
  rail_pick_and_place_msgs::GraspWithSuccessRate toROSGraspWithSuccessRateMessage() const;

private:
  /*! The ID of the associated grasp model. */
  uint32_t grasp_model_id_;
  /*! The name of the object and end effector frame identifier for this demonstration entry. */
  std::string eef_frame_id_;
  /*! The grasp pose data. */
  Pose grasp_pose_;
  /*! The success rate information. */
  uint32_t successes_, attempts_;
};

}
}
}

#endif
