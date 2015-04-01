/*!
 * \file Orientation.h
 * \brief Quaternion orientation information.
 *
 * An orientation simply contains x, y, z, and w values. This class is useful for internal data management within
 * the graspdb library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASPDB_ORIENTATION_H_
#define RAIL_PICK_AND_PLACE_GRASPDB_ORIENTATION_H_

// ROS
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

// C++ Standard Library
#include <string>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

/*!
 * \class Orientation
 * \brief Quaternion orientation information.
 *
 * An orientation simply contains x, y, z, and w values. This class is useful for internal data management within
 * the graspdb library. Convenience functions are added for use with ROS messages.
 */
class Orientation
{
public:
  /*!
   * \brief Create a new Orientation.
   *
   * Creates a new Orientation with the given x, y, z, and w values (defaults are 0).
   *
   * \param x The x value (default of 0).
   * \param y The y value (default of 0).
   * \param z The z value (default of 0).
   * \param w The w value (default of 0).
   */
  Orientation(const double x = 0, const double y = 0, const double z = 0, const double w = 0);

  /*!
   * \brief Create a new Orientation.
   *
   * Creates a new Orientation with the given x, y, z, and w values from the ROS Quaternion message.
   *
   * \param point The ROS Quaternion message to extract values from.
   */
  Orientation(const geometry_msgs::Quaternion &quaternion);

  /*!
   * \brief Create a new Orientation.
   *
   * Creates a new Orientation with the given x, y, z, and w values from the ROS tf2 Quaternion.
   *
   * \param point The ROS tf2 Quaternion to extract values from.
   */
  Orientation(const tf2::Quaternion &quaternion);

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
   * \brief W value mutator.
   *
   * Set the w value of this Position.
   *
   * \param w The new w value.
   */
  void setW(const double w);

  /*!
   * \brief W value accessor.
   *
   * Get the w value of this Position.
   *
   * \return The w value.
   */
  double getW() const;

  /*!
   * Converts this Orientation object into a ROS Quaternion message.
   *
   * \return The ROS Quaternion message with this orientation data.
   */
  geometry_msgs::Quaternion toROSQuaternionMessage() const;

  /*!
   * Converts this Orientation object into a ROS tf2 Quaternion.
   *
   * \return The ROS tf2 Quaternion with this orientation data.
   */
  tf2::Quaternion toTF2Quaternion() const;

  /*!
   * Converts this Orientation object into a ROS tf2 Matrix3x3.
   *
   * \return The ROS tf2 Matrix3x3 with this orientation data.
   */
  tf2::Matrix3x3 toTF2Matrix3x3() const;

private:
  /*! Members to hold values. */
  double x_, y_, z_, w_;
};

}
}
}

#endif
