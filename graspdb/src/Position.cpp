#include <graspdb/Position.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

Position::Position(const double x, const double y, const double z)
{
  // set position data
  x_ = x;
  y_ = y;
  z_ = z;
}

void Position::setX(const double x)
{
  x_ = x;
}

double Position::getX() const
{
  return x_;
}

void Position::setY(const double y)
{
  y_ = y;
}

double Position::getY() const
{
  return y_;
}

void Position::setZ(const double z)
{
  z_ = z;
}

double Position::getZ() const
{
  return z_;
}

