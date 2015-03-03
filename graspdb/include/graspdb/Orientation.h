#ifndef RAIL_GRASPDB_ORIENTATION_H_
#define RAIL_GRASPDB_ORIENTATION_H_

#include <string>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

class Orientation
{
public:
  Orientation(double x = 0, double y = 0, double z = 0, double w = 0);

  void setX(double x);

  double getX() const;

  void setY(double y);

  double getY() const;

  void setZ(double z);

  double getZ() const;

  void setW(double w);

  double getW() const;

protected:
  double x_, y_, z_, w_;
};

}
}
}

#endif
