/*!
 * \file Position.h
 * \brief 3-point position information.
 *
 * A position simply contains x, y, and z value. This class is useful for internal data management within the graspdb
 * library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASPDB_POSITION_H_
#define RAIL_PICK_AND_PLACE_GRASPDB_POSITION_H_

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Vector3.h>

// C++ Standard Library
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
 * A position simply contains x, y, and z value. This class is useful for internal data management within the graspdb
 * library. Convenience functions are added for use with ROS messages.
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
   * \brief Create a new Position.
   *
   * Creates a new Position with the given x, y, and z values from the ROS tf2 Vector3.
   *
   * \param point The ROS tf2 Vector3 to extract values from.
   */
  Position(const tf2::Vector3 &v);

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

  /*!
   * Converts this Position object into a ROS Point message.
   *
   * \return The ROS Point message with this position data.
   */
  geometry_msgs::Point toROSPointMessage() const;

  /*!
   * Converts this Position object into a ROS Vector3 message.
   *
   * \return The ROS Vector3 message with this position data.
   */
  geometry_msgs::Vector3 toROSVector3Message() const;

  /*!
   * Converts this Position object into a ROS tf2 Vector3.
   *
   * \return The ROS tf2 Vector3 with this position data.
   */
  tf2::Vector3 toTF2Vector3() const;

private:
  /*! Members to hold values. */
  double x_, y_, z_;
};

}
}
}

#endif
