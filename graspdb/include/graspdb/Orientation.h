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
  Orientation(const double x = 0, const double y = 0, const double z = 0, const double w = 0);

  void setX(const double x);

  double getX() const;

  void setY(const double y);

  double getY() const;

  void setZ(const double z);

  double getZ() const;

  void setW(const double w);

  double getW() const;

protected:
  double x_, y_, z_, w_;
};

}
}
}

#endif
