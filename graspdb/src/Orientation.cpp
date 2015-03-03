#include <graspdb/Orientation.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

Orientation::Orientation(double x, double y, double z, double w)
{
  // set orientation data
  x_ = x;
  y_ = y;
  z_ = z;
  w_ = w;
}

void Orientation::setX(double x)
{
  x_ = x;
}

double Orientation::getX() const
{
  return x_;
}

void Orientation::setY(double y)
{
  y_ = y;
}

double Orientation::getY() const
{
  return y_;
}

void Orientation::setZ(double z)
{
  z_ = z;
}

double Orientation::getZ() const
{
  return z_;
}

void Orientation::setW(double w)
{
  w_ = w;
}

double Orientation::getW() const
{
  return w_;
}

