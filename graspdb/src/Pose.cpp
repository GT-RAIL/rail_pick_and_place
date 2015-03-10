/*!
 * \file Pose.cpp
 * \brief Position and orientation information with respect to a given coordinate frame.
 *
 * A pose contains position and orientation information as well as a coordinate frame identifier.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

#include <graspdb/Pose.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

Pose::Pose(const string fixed_frame_id, const string grasp_frame_id, const Position position,
    const Orientation orientation)
    : fixed_frame_id_(fixed_frame_id), grasp_frame_id_(grasp_frame_id), position_(position), orientation_(orientation)
{
}

Pose::Pose(const std::string fixed_frame_id, const string grasp_frame_id, const geometry_msgs::Point &position,
    const geometry_msgs::Quaternion &orientation)
    : fixed_frame_id_(fixed_frame_id), grasp_frame_id_(grasp_frame_id), position_(position), orientation_(orientation)
{
}

Pose::Pose(const std::string fixed_frame_id, const string grasp_frame_id, const geometry_msgs::Pose &pose)
    : fixed_frame_id_(fixed_frame_id),
      grasp_frame_id_(grasp_frame_id),
      position_(pose.position),
      orientation_(pose.orientation)
{
}

Pose::Pose(const std::string fixed_frame_id, const string grasp_frame_id, const geometry_msgs::Transform &transform)
    : fixed_frame_id_(fixed_frame_id),
      grasp_frame_id_(grasp_frame_id),
      position_(transform.translation),
      orientation_(transform.rotation)
{
}

Pose::Pose(const std::string fixed_frame_id, const string grasp_frame_id, const geometry_msgs::Vector3 &position,
    const geometry_msgs::Quaternion &orientation)
    : fixed_frame_id_(fixed_frame_id), grasp_frame_id_(grasp_frame_id), position_(position), orientation_(orientation)
{
}

void Pose::setFixedFrameID(const std::string fixed_frame_id)
{
  fixed_frame_id_ = fixed_frame_id;
}

const string &Pose::getFixedFrameID() const
{
  return fixed_frame_id_;
}

void Pose::setGraspFrameID(const std::string grasp_frame_id)
{
  grasp_frame_id_ = grasp_frame_id;
}

const string &Pose::getGraspFrameID() const
{
  return grasp_frame_id_;
}

void Pose::setPosition(const Position position)
{
  position_ = position;
}

const Position &Pose::getPosition() const
{
  return position_;
}

void Pose::setOrientation(const Orientation orientation)
{
  orientation_ = orientation;
}

const Orientation &Pose::getOrientation() const
{
  return orientation_;
}

geometry_msgs::Pose Pose::toROSPoseMessage() const
{
  geometry_msgs::Pose p;
  p.position = position_.toROSPointMessage();
  p.orientation = orientation_.toROSQuaternionMessage();
  return p;
}

geometry_msgs::PoseStamped Pose::toROSPoseStampedMessage() const
{
  geometry_msgs::PoseStamped p;
  p.header.frame_id = fixed_frame_id_;
  p.pose.position = position_.toROSPointMessage();
  p.pose.orientation = orientation_.toROSQuaternionMessage();
  return p;
}
