/*!
 * \file Position.h
 * \brief 3-point position information.
 *
 * A position simply contains x, y, and z value.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

#ifndef RAIL_GRASPDB_POSITION_H_
#define RAIL_GRASPDB_POSITION_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <string>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

/*!
 * \class Position
 * \brief 3-point position information.
 *
 * A position simply contains an x, y, and z value.
 */
class Position
{
public:
  /*!
   * \brief Create a new Position.
   *
   * Creates a new Position with the given x, y, and z values (defaults are 0).
   *
   * \param x The x value (default of 0).
   * \param y The y value (default of 0).
   * \param z The z value (default of 0).
   */
  Position(const double x = 0, const double y = 0, const double z = 0);

  /*!
   * \brief Create a new Position.
   *
   * Creates a new Position with the given x, y, and z values from the ROS Point message.
   *
   * \param point The ROS Point message to extract values from.
   */
  Position(const geometry_msgs::Point &point);

  /*!
   * \brief Create a new Position.
   *
   * Creates a new Position with the given x, y, and z values from the ROS Vector3 message.
   *
   * \param point The ROS Vector3 message to extract values from.
   */
  Position(const geometry_msgs::Vector3 &v);

  /*!
   * \brief X value mutator.
   *
   * Set the x value of this Position.
   *
   * \param x The new x value.
   */
  void setX(const double x);

  /*!
   * \brief X value accessor.
   *
   * Get the x value of this Position.
   *
   * \return The x value.
   */
  double getX() const;

  /*!
   * \brief Y value mutator.
   *
   * Set the y value of this Position.
   *
   * \param y The new y value.
   */
  void setY(const double y);

  /*!
   * \brief Y value accessor.
   *
   * Get the y value of this Position.
   *
   * \return The y value.
   */
  double getY() const;

  /*!
   * \brief Z value mutator.
   *
   * Set the z value of this Position.
   *
   * \param z The new z value.
   */
  void setZ(const double z);

  /*!
   * \brief Z value accessor.
   *
   * Get the z value of this Position.
   *
   * \return The z value.
   */
  double getZ() const;

private:
  /*! Members to hold values. */
  double x_, y_, z_;
};

}
}
}

#endif
