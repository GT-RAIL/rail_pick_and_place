#include <graspdb/Pose.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

Pose::Pose(string frame_id, Position position, Orientation orientation)
    : frame_id_(frame_id), position_(position), orientation_(orientation)
{
}

void Pose::setFrameID(std::string frame_id)
{
  frame_id_ = frame_id;
}

string &Pose::getFrameID()
{
  return frame_id_;
}

void Pose::setPosition(Position position)
{
  position_ = position;
}

Position &Pose::getPosition()
{
  return position_;
}

void Pose::setOrientation(Orientation orientation)
{
  orientation_ = orientation;
}

Orientation &Pose::getOrientation()
{
  return orientation_;
}
