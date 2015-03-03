#ifndef RAIL_GRASPDB_POSITION_H_
#define RAIL_GRASPDB_POSITION_H_

#include <string>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

class Position
{
public:
  Position(const double x = 0, const double y = 0, const double z = 0);

  void setX(const double x);

  double getX() const;

  void setY(const double y);

  double getY() const;

  void setZ(const double z);

  double getZ() const;

private:
  double x_, y_, z_;
};

}
}
}

#endif
