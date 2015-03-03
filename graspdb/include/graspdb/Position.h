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
  Position(double x = 0, double y = 0, double z = 0);

  void setX(double x);

  double getX() const;

  void setY(double y);

  double getY() const;

  void setZ(double z);

  double getZ() const;

private:
  double x_, y_, z_;
};

}
}
}

#endif
