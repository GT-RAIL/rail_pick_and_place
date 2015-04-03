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

class PCLGraspModel : public graspdb::GraspModel
{
public:
  PCLGraspModel(const graspdb::GraspModel &grasp_model = graspdb::GraspModel());

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

  graspdb::GraspModel toGraspModel() const;

private:
  /*! The internal shared pointer to the PCL point cloud. */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_;
};

}
}

#endif
