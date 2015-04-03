/*!
 * \file Pose.cpp
 * \brief Position and orientation information with respect to a given coordinate frame.
 *
 * A pose contains position and orientation information as well as a coordinate frame identifier. This coordinate
 * frame should be a fixed frame on the robot. This class is useful for internal data management within the graspdb
 * library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

// graspdb
#include "graspdb/Pose.h"

using namespace std;
using namespace rail::pick_and_place::graspdb;

Pose::Pose(const string &robot_fixed_frame_id, const Position &position, const Orientation &orientation)
    : robot_fixed_frame_id_(robot_fixed_frame_id), position_(position), orientation_(orientation)
{
}

Pose::Pose(const string &robot_fixed_frame_id, const geometry_msgs::Pose &pose)
    : robot_fixed_frame_id_(robot_fixed_frame_id), position_(pose.position), orientation_(pose.orientation)
{
}

Pose::Pose(const geometry_msgs::PoseStamped &pose)
    : robot_fixed_frame_id_(pose.header.frame_id), position_(pose.pose.position), orientation_(pose.pose.orientation)
{
}

Pose::Pose(const string &robot_fixed_frame_id, const geometry_msgs::PoseWithCovariance &pose)
    : robot_fixed_frame_id_(robot_fixed_frame_id), position_(pose.pose.position), orientation_(pose.pose.orientation)
{
}

Pose::Pose(const geometry_msgs::PoseWithCovarianceStamped &pose)
    : robot_fixed_frame_id_(pose.header.frame_id), position_(pose.pose.pose.position),
      orientation_(pose.pose.pose.orientation)
{
}

Pose::Pose(const string &robot_fixed_frame_id, const geometry_msgs::Transform &tf)
    : robot_fixed_frame_id_(robot_fixed_frame_id), position_(tf.translation), orientation_(tf.rotation)
{
}

Pose::Pose(const geometry_msgs::TransformStamped &tf)
    : robot_fixed_frame_id_(tf.header.frame_id), position_(tf.transform.translation),
      orientation_(tf.transform.rotation)
{
}

Pose::Pose(const std::string &robot_fixed_frame_id, const tf2::Transform &tf)
    : robot_fixed_frame_id_(robot_fixed_frame_id), position_(tf.getOrigin()), orientation_(tf.getRotation())
{
}

void Pose::setRobotFixedFrameID(const string &robot_fixed_frame_id)
{
  robot_fixed_frame_id_ = robot_fixed_frame_id;
}

const string &Pose::getRobotFixedFrameID() const
{
  return robot_fixed_frame_id_;
}

void Pose::setPosition(const Position &position)
{
  position_ = position;
}

const Position &Pose::getPosition() const
{
  return position_;
}

Position &Pose::getPosition()
{
  return position_;
}

void Pose::setOrientation(const Orientation &orientation)
{
  orientation_ = orientation;
}

const Orientation &Pose::getOrientation() const
{
  return orientation_;
}

Orientation &Pose::getOrientation()
{
  return orientation_;
}

geometry_msgs::Pose Pose::toROSPoseMessage() const
{
  geometry_msgs::Pose pose;
  pose.position = position_.toROSPointMessage();
  pose.orientation = orientation_.toROSQuaternionMessage();
  return pose;
}

geometry_msgs::PoseStamped Pose::toROSPoseStampedMessage() const
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = robot_fixed_frame_id_;
  pose.pose.position = position_.toROSPointMessage();
  pose.pose.orientation = orientation_.toROSQuaternionMessage();
  return pose;
}

tf2::Transform Pose::toTF2Transform() const
{
  tf2::Quaternion orientation = orientation_.toTF2Quaternion();
  tf2::Vector3 translation = position_.toTF2Vector3();
  tf2::Transform tf(orientation, translation);
  return tf;
}
