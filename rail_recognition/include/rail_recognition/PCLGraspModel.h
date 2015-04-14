/*!
 * \file PCLGraspModel.h
 * \brief A wrapper around a graspdb GrapsModel with a PCL point cloud.
 *
 * The PCLGraspModel is simply a wrapper around the graspdb GrapsModel with a PCL point cloud. A flag can also be set
 * to mark the grasp model as an original (as apposed to newly generated during model generation). All accessors to
 * the ROS point cloud message are disabled.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_PCL_GRASP_MODEL_H_
#define RAIL_PICK_AND_PLACE_PCL_GRASP_MODEL_H_

// ROS
#include <graspdb/GraspModel.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rail
{
namespace pick_and_place
{

/*!
 * \class PCLGraspModel
 * \brief A wrapper around a graspdb GrapsModel with a PCL point cloud.
 *
 * The PCLGraspModel is simply a wrapper around the graspdb GrapsModel with a PCL point cloud. A flag can also be set
 * to mark the grasp model as an original (as apposed to newly generated during model generation). All accessors to
 * the ROS point cloud message are disabled.
 */
class PCLGraspModel : public graspdb::GraspModel
{
public:
  /*!
   * \brief Creates a new PCLGraspModel.
   *
   * Creates a new ObjectRecognizer from the graspdb grasp model object. The point cloud is converted during
   * construction. The original flag defaults to false.
   *
   * \param grasp_model The graspdb GraspModel to create a PCLGraspModel from (defaults to an empty GraspModel).
   */
  PCLGraspModel(const graspdb::GraspModel &grasp_model = graspdb::GraspModel());

  /*!
   * \brief Original flag accessor.
   *
   * Gets the value of the original flag (defaults to false).
   *
   * \return The value of the original model flag.
   */
  bool isOriginal() const;

  /*!
   * \brief Original flag mutator.
   *
   * Sets the value of the original flag.
   *
   * \param original The new value of the original model flag.
   */
  void setOriginal(const bool original);

  /*!
   * \brief PCL point cloud accessor.
   *
   * Get the PCL point cloud.
   *
   * \return The PCL point cloud.
   */
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &getPCLPointCloud() const;

  /*!
   * \brief Disabled point cloud accessor (immutable).
   *
   * Unavailable function.
   *
   * \return Nothing (exception will be thrown).
   * \throws std::runtime_error Thrown since this method is not supported.
   */
  const sensor_msgs::PointCloud2 &getPointCloud() const;

  /*!
   * \brief Disabled point cloud accessor.
   *
   * Unavailable function.
   *
   * \return Nothing (exception will be thrown).
   * \throws std::runtime_error Thrown since this method is not supported.
   */
  sensor_msgs::PointCloud2 &getPointCloud();

  /*!
   * \brief Point cloud mutator.
   *
   * Set the point cloud message to the given values based on the ROS message.
   *
   * \param point_cloud The ROS PointCloud2 message to store.
   */
  void setPointCloud(const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Average point cloud red value accessor.
   *
   * Get average point cloud red value.
   *
   * \return The average point cloud red value.
   */
  double getAverageRed() const;

  /*!
   * \brief Average point cloud green value accessor.
   *
   * Get average point cloud green value.
   *
   * \return The average point cloud green value.
   */
  double getAverageGreen() const;

  /*!
   * \brief Average point cloud blue value accessor.
   *
   * Get average point cloud blue value.
   *
   * \return The average point cloud blue value.
   */
  double getAverageBlue() const;

  /*!
   * \brief Creates a graspdb GraspModel from this PCL grasp model.
   *
   * Creates and returns a new graspdb GraspModel from this PCL grasp model.
   *
   * \return The new graspdb GraspModel.
   */
  graspdb::GraspModel toGraspModel() const;

private:
  /*! The original model flag. */
  bool original_;
  /*! The internal shared pointer to the PCL point cloud. */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_;
  /*! Color information for the point cloud */
  double avg_r_, avg_g_, avg_b_;
};

}
}

#endif
