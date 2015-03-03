#ifndef RAIL_GRASPDB_POSE_H_
#define RAIL_GRASPDB_POSE_H_

#include <string>
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
  Pose(std::string frame_id = "", Position position = Position(), Orientation orientation = Orientation());

  void setFrameID(std::string frame_id);

  std::string &getFrameID();

  void setPosition(Position position);

  Position &getPosition();

  void setOrientation(Orientation orientation);

  Orientation &getOrientation();

private:
  std::string frame_id_;
  Position position_;
  Orientation orientation_;
};

}
}
}

#endif
