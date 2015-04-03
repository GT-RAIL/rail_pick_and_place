/*!
 * \file Pose.h
 * \brief Position and orientation information with respect to a given coordinate frame.
 *
 * A pose contains position and orientation information as well as a coordinate frame identifier. This coordinate
 * frame should be a fixed frame on the robot. This class is useful for internal data management within the graspdb
 * library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASPDB_POSE_H_
#define RAIL_PICK_AND_PLACE_GRASPDB_POSE_H_

// graspdb
#include "Orientation.h"
#include "Position.h"

// ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>

// C++ Standard Library
#include <string>

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
 * A pose contains position and orientation information as well as a coordinate frame identifier. This coordinate
 * frame should be a fixed frame on the robot. This class is useful for internal data management within the graspdb
 * library. Convenience functions are added for use with ROS messages.
 */
class Pose
{
public:
  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given frame identifier, position, and orientation (defaults are 0).
   *
   * \param robot_fixed_frame_id The reference frame identifier (defaults to the empty string).
   * \param position The position values (defaults are 0).
   * \param orientation The orientation values (defaults are 0).
   */
  Pose(const std::string &robot_fixed_frame_id = "", const Position &position = Position(),
      const Orientation &orientation = Orientation());

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given frame identifier and pose data from the given ROS message.
   *
   * \param robot_fixed_frame_id The reference frame identifier.
   * \param pose The ROS Pose message to extract position and orientation data from.
   */
  Pose(const std::string &robot_fixed_frame_id, const geometry_msgs::Pose &pose);

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given pose data from the given ROS PoseStamped message. Timestamp information is
   * ignored from the header.
   *
   * \param pose The ROS PoseStamped message to extract data from.
   */
  Pose(const geometry_msgs::PoseStamped &pose);

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given pose data from the given frame identifier and ROS PoseWithCovariance message.
   * Covariance information is ignored.
   *
   * \param robot_fixed_frame_id The reference frame identifier.
   * \param pose The ROS PoseWithCovariance message to extract data from.
   */
  Pose(const std::string &robot_fixed_frame_id, const geometry_msgs::PoseWithCovariance &pose);

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given pose data from the given frame identifier and ROS PoseWithCovarianceStamped
   * message. Timestamp and covariance information is ignored.
   *
   * \param pose The ROS PoseWithCovariance message to extract data from.
   */
  Pose(const geometry_msgs::PoseWithCovarianceStamped &pose);

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given frame identifier and positional data from the given ROS Transform message.
   *
   * \param robot_fixed_frame_id The reference frame identifier.
   * \param position The ROS Transform message to extract position and orientation data from.
   */
  Pose(const std::string &robot_fixed_frame_id, const geometry_msgs::Transform &tf);

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the positional data from the given ROS TransformStamped message. The frame ID is
   * taken from the header (the parent frame). Timestamp information and child frame ID is ignored.
   *
   * \param position The ROS TransformStamped message to extract data from.
   */
  Pose(const geometry_msgs::TransformStamped &tf);

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given frame identifier and positional data from the given ROS tf2 Transform.
   *
   * \param robot_fixed_frame_id The reference frame identifier.
   * \param position The ROS tf2 Transform to extract position and orientation data from.
   */
  Pose(const std::string &robot_fixed_frame_id, const tf2::Transform &tf);

  /*!
   * \brief Frame ID value mutator.
   *
   * Set the frame ID value of this Pose.
   *
   * \param frame_id The new frame ID value.
   */
  void setRobotFixedFrameID(const std::string &robot_fixed_frame_id);

  /*!
   * \brief Robot fixed frame ID value accessor.
   *
   * Get the robot fixed frame ID value of this Pose.
   *
   * \return The robot fixed frame ID value.
   */
  const std::string &getRobotFixedFrameID() const;

  /*!
   * \brief Position value mutator.
   *
   * Set the position value of this Pose.
   *
   * \param position The new position value.
   */
  void setPosition(const Position &position);

  /*!
   * \brief Position value accessor.
   *
   * Get the position value of this Pose.
   *
   * \return The position value.
   */
  const Position &getPosition() const;

  /*!
   * \brief Position value accessor (immutable).
   *
   * Get the position value of this Pose.
   *
   * \return The position value.
   */
  Position &getPosition();

  /*!
   * \brief Orientation value mutator.
   *
   * Set the orientation value of this Pose.
   *
   * \param orientation The new orientation value.
   */
  void setOrientation(const Orientation &orientation);

  /*!
   * \brief Orientation value accessor (immutable).
   *
   * Get the orientation value of this Pose.
   *
   * \return The orientation value.
   */
  const Orientation &getOrientation() const;

  /*!
   * \brief Orientation value accessor.
   *
   * Get the orientation value of this Pose.
   *
   * \return The orientation value.
   */
  Orientation &getOrientation();

  /*!
   * Converts this Pose object into a ROS Pose message.
   *
   * \return The ROS Pose message with this pose data.
   */
  geometry_msgs::Pose toROSPoseMessage() const;

  /*!
   * Converts this Pose object into a ROS PoseStamped message. Timestamp information is set to 0.
   *
   * \return The ROS PoseStamped message with this pose data.
   */
  geometry_msgs::PoseStamped toROSPoseStampedMessage() const;

  /*!
   * Converts this Pose object into a ROS Transform message.
   *
   * \return The ROS Transform message with this positional data.
   */
  geometry_msgs::Transform toROSTransformMessage() const;

  /*!
   * Converts this Pose object into a ROS tf2 Transform.
   *
   * \return The ROS tf2 Transform with this positional data.
   */
  tf2::Transform toTF2Transform() const;

private:
  /*! Frame identifier. */
  std::string robot_fixed_frame_id_;
  /*! Position data. */
  Position position_;
  /*! Orientation data. */
  Orientation orientation_;
};

}
}
}

#endif
