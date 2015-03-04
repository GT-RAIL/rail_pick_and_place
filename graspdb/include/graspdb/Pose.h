/*!
 * \file Pose.h
 * \brief Position and orientation information with respect to a given coordinate frame.
 *
 * A pose contains position and orientation information as well as a coordinate frame identifier.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

#ifndef RAIL_GRASPDB_POSE_H_
#define RAIL_GRASPDB_POSE_H_

#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <graspdb/Position.h>
#include <graspdb/Orientation.h>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

/*!
 * \class Pose
 * \brief Position and orientation information with respect to a given coordinate frame.
 *
 * A pose contains position and orientation information as well as a coordinate frame identifier.
 */
class Pose
{
public:
  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given frame identifier, position, and orientation (defaults are 0).
   *
   * \param fixed_frame_id The fixed (reference) frame identifier (defaults to the empty string).
   * \param grasp_frame_id The grasp frame identifier (defaults to the empty string).
   * \param position The position values (defaults are 0).
   * \param orientation The orientation values (defaults are 0).
   */
  Pose(const std::string fixed_frame_id = "", const std::string grasp_frame_id = "",
      const Position position = Position(), const Orientation orientation = Orientation());

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given frame identifier, position, and orientation from ROS messages.
   *
   * \param fixed_frame_id The fixed (reference) frame identifier.
   * \param grasp_frame_id The grasp frame identifier.
   * \param position The ROS Point message to extract position data from.
   * \param orientation The ROS Quaternion message to extract orientation data from.
   */
  Pose(const std::string fixed_frame_id, const std::string grasp_frame_id, const geometry_msgs::Point &position,
      const geometry_msgs::Quaternion &orientation);

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given frame identifier and pose data from the given ROS message.
   *
   * \param fixed_frame_id The fixed (reference) frame identifier.
   * \param grasp_frame_id The grasp frame identifier.
   * \param position The ROS Pose message to extract position and orientation data from.
   */
  Pose(const std::string fixed_frame_id, const std::string grasp_frame_id, const geometry_msgs::Pose &pose);

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given frame identifier and pose data from the given ROS message.
   *
   * \param fixed_frame_id The fixed (reference) frame identifier.
   * \param grasp_frame_id The grasp frame identifier.
   * \param position The ROS Transform message to extract position and orientation data from.
   */
  Pose(const std::string fixed_frame_id, const std::string grasp_frame_id, const geometry_msgs::Transform &transform);

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given frame identifier, position, and orientation from ROS messages.
   *
   * \param fixed_frame_id The fixed (reference) frame identifier.
   * \param grasp_frame_id The grasp frame identifier.
   * \param position The ROS Vector3 message to extract position data from.
   * \param orientation The ROS Quaternion message to extract orientation data from.
   */
  Pose(const std::string fixed_frame_id, const std::string grasp_frame_id, const geometry_msgs::Vector3 &position,
      const geometry_msgs::Quaternion &orientation);

  /*!
   * \brief Fixed frame ID value mutator.
   *
   * Set the fixed frame ID value of this Pose.
   *
   * \param fixed_frame_id The new frame ID value.
   */
  void setFixedFrameID(const std::string fixed_frame_id);

  /*!
   * \brief Fixed frame ID value accessor.
   *
   * Get the fixed frame ID value of this Pose.
   *
   * \return The fixed frame ID value.
   */
  const std::string &getFixedFrameID() const;

  /*!
   * \brief Grasp frame ID value mutator.
   *
   * Set the grasp frame ID value of this Pose.
   *
   * \param grasp_frame_id The new frame ID value.
   */
  void setGraspFrameID(const std::string grasp_frame_id);

  /*!
   * \brief Grasp frame ID value accessor.
   *
   * Get the grasp frame ID value of this Pose.
   *
   * \return The grasp frame ID value.
   */
  const std::string &getGraspFrameID() const;

  /*!
   * \brief Position value mutator.
   *
   * Set the position value of this Pose.
   *
   * \param position The new position value.
   */
  void setPosition(const Position position);

  /*!
   * \brief Position value accessor.
   *
   * Get the position value of this Pose.
   *
   * \return The position value.
   */
  const Position &getPosition() const;

  /*!
   * \brief Orientation value mutator.
   *
   * Set the orientation value of this Pose.
   *
   * \param orientation The new orientation value.
   */
  void setOrientation(const Orientation orientation);

  /*!
   * \brief Orientation value accessor.
   *
   * Get the orientation value of this Pose.
   *
   * \return The orientation value.
   */
  const Orientation &getOrientation() const;

  /*!
   * Converts this Pose object into a ROS Pose message.
   *
   * \return The ROS Pose message with this pose data.
   */
  geometry_msgs::Pose toROSPoseMessage() const;

  /*!
   * Converts this Pose object into a ROS PoseStamped message. The frame_id field is set to the Pose's fixed frame.
   *
   * \return The ROS PoseStamped message with this pose data.
   */
  geometry_msgs::PoseStamped toROSPoseStampedMessage() const;

private:
  /*! Frame identifiers. */
  std::string fixed_frame_id_, grasp_frame_id_;
  /*! Position data. */
  Position position_;
  /*! Orientation data. */
  Orientation orientation_;
};

}
}
}

#endif
