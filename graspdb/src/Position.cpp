/*!
 * \file Position.cpp
 * \brief 3-point position information.
 *
 * A position simply contains x, y, and z value. This class is useful for internal data management within the graspdb
 * library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

// graspdb
#include "graspdb/Position.h"

using namespace std;
using namespace rail::pick_and_place::graspdb;

Position::Position(const double x, const double y, const double z)
{
  // set position data
  x_ = x;
  y_ = y;
  z_ = z;
}

Position::Position(const geometry_msgs::Point &point)
{
  // copy position data
  x_ = point.x;
  y_ = point.y;
  z_ = point.z;
}

Position::Position(const geometry_msgs::Vector3 &v)
{
  // copy position data
  x_ = v.x;
  y_ = v.y;
  z_ = v.z;
}

Position::Position(const tf2::Vector3 &v)
{
  // copy position data
  x_ = v.getX();
  y_ = v.getY();
  z_ = v.getZ();
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

geometry_msgs::Point Position::toROSPointMessage() const
{
  geometry_msgs::Point p;
  p.x = x_;
  p.y = y_;
  p.z = z_;
  return p;
}

geometry_msgs::Vector3 Position::toROSVector3Message() const
{
  geometry_msgs::Vector3 v;
  v.x = x_;
  v.y = y_;
  v.z = z_;
  return v;
}

tf2::Vector3 Position::toTF2Vector3() const
{
  tf2::Vector3 v(x_, y_, z_);
  return v;
}

