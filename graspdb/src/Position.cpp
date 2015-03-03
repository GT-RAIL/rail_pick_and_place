#include <graspdb/Position.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

Position::Position(double x, double y, double z)
{
  // set position data
  x_ = x;
  y_ = y;
  z_ = z;
}

void Position::setX(double x)
{
  x_ = x;
}

double Position::getX() const
{
  return x_;
}

void Position::setY(double y)
{
  y_ = y;
}

double Position::getY() const
{
  return y_;
}

void Position::setZ(double z)
{
  z_ = z;
}

double Position::getZ() const
{
  return z_;
}

