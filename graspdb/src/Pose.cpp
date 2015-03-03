#include <graspdb/Pose.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

Pose::Pose(const string frame_id, const Position position, const Orientation orientation)
    : frame_id_(frame_id), position_(position), orientation_(orientation)
{
}

Pose::Pose(const std::string frame_id, const geometry_msgs::Point &position, const geometry_msgs::Quaternion &orientation)
    : frame_id_(frame_id), position_(position), orientation_(orientation)
{
}

Pose::Pose(const std::string frame_id, const geometry_msgs::Pose &pose)
    : frame_id_(frame_id), position_(pose.position), orientation_(pose.orientation)
{
}

Pose::Pose(const std::string frame_id, const geometry_msgs::Transform &transform)
    : frame_id_(frame_id), position_(transform.translation), orientation_(transform.rotation)
{
}

Pose::Pose(const std::string frame_id, const geometry_msgs::Vector3 &position, const geometry_msgs::Quaternion &orientation)
    : frame_id_(frame_id), position_(position), orientation_(orientation)
{
}

void Pose::setFrameID(const std::string frame_id)
{
  frame_id_ = frame_id;
}

const string &Pose::getFrameID() const
{
  return frame_id_;
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
