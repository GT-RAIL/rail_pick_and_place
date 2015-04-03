/*!
 * \file Orientation.cpp
 * \brief Quaternion orientation information.
 *
 * An orientation simply contains x, y, z, and w values. This class is useful for internal data management within
 * the graspdb library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

// graspdb
#include "graspdb/Orientation.h"

using namespace std;
using namespace rail::pick_and_place::graspdb;

Orientation::Orientation(const double x, const double y, const double z, const double w)
{
  // set orientation data
  x_ = x;
  y_ = y;
  z_ = z;
  w_ = w;
}

Orientation::Orientation(const geometry_msgs::Quaternion &quaternion)
{
  // copy position data
  x_ = quaternion.x;
  y_ = quaternion.y;
  z_ = quaternion.z;
  w_ = quaternion.w;
}

Orientation::Orientation(const tf2::Quaternion &quaternion)
{
  // copy position data
  x_ = quaternion.getX();
  y_ = quaternion.getY();
  z_ = quaternion.getZ();
  w_ = quaternion.getW();
}

void Orientation::setX(const double x)
{
  x_ = x;
}

double Orientation::getX() const
{
  return x_;
}

void Orientation::setY(const double y)
{
  y_ = y;
}

double Orientation::getY() const
{
  return y_;
}

void Orientation::setZ(const double z)
{
  z_ = z;
}

double Orientation::getZ() const
{
  return z_;
}

void Orientation::setW(const double w)
{
  w_ = w;
}

double Orientation::getW() const
{
  return w_;
}

geometry_msgs::Quaternion Orientation::toROSQuaternionMessage() const
{
  geometry_msgs::Quaternion q;
  q.x = x_;
  q.y = y_;
  q.z = z_;
  q.w = w_;
  return q;
}

tf2::Quaternion Orientation::toTF2Quaternion() const
{
  tf2::Quaternion q(x_, y_, z_, w_);
  return q;
}

tf2::Matrix3x3 Orientation::toTF2Matrix3x3() const
{
  tf2::Quaternion q = this->toTF2Quaternion();
  tf2::Matrix3x3 m(q);
  return m;
}
