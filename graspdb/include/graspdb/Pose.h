#ifndef RAIL_GRASPDB_POSE_H_
#define RAIL_GRASPDB_POSE_H_

#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <graspdb/Position.h>
#include <graspdb/Orientation.h>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

class Pose
{
public:
  Pose(const std::string frame_id = "", const Position position = Position(), const Orientation orientation = Orientation());

  Pose(const std::string frame_id, const geometry_msgs::Point &position, const geometry_msgs::Quaternion &orientation);

  Pose(const std::string frame_id, const geometry_msgs::Pose &pose);

  Pose(const std::string frame_id, const geometry_msgs::Transform &transform);

  Pose(const std::string frame_id, const geometry_msgs::Vector3 &position, const geometry_msgs::Quaternion &orientation);

  void setFrameID(const std::string frame_id);

  const std::string &getFrameID() const;

  void setPosition(const Position position);

  const Position &getPosition() const;

  void setOrientation(const Orientation orientation);

  const Orientation &getOrientation() const;

private:
  std::string frame_id_;
  Position position_;
  Orientation orientation_;
};

}
}
}

#endif
