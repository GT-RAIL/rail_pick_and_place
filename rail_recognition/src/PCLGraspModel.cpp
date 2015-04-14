/*!
 * \file PCLGraspModel.cpp
 * \brief A wrapper around a graspdb GrapsModel with a PCL point cloud.
 *
 * The PCLGraspModel is simply a wrapper around the graspdb GrapsModel with a PCL point cloud. A flag can also be set
 * to mark the grasp model as an original (as apposed to newly generated during model generation). All accessors to
 * the ROS point cloud message are disabled.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

// RAIL Recognition
#include "rail_recognition/PCLGraspModel.h"
#include "rail_recognition/PointCloudMetrics.h"

using namespace std;
using namespace rail::pick_and_place;

PCLGraspModel::PCLGraspModel(const graspdb::GraspModel &grasp_model)
    : graspdb::GraspModel(grasp_model),
      pc_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  // default to false
  original_ = false;

  // copy the point cloud if it exists
  if (grasp_model.getPointCloud().data.size() > 0)
  {
    this->setPointCloud(grasp_model.getPointCloud());
  } else
  {
    // simply store the header information
    pc_->header.frame_id = grasp_model.getPointCloud().header.frame_id;
    avg_r_ = 0;
    avg_g_ = 0;
    avg_b_ = 0;
  }
}

bool PCLGraspModel::isOriginal() const
{
  return original_;
}

void PCLGraspModel::setOriginal(const bool original)
{
  original_ = original;
}

const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &PCLGraspModel::getPCLPointCloud() const
{
  return pc_;
}

sensor_msgs::PointCloud2 &PCLGraspModel::getPointCloud()
{
  throw std::runtime_error("PCLGraspModel::getPointCloud() is unsupported.");
}

const sensor_msgs::PointCloud2 &PCLGraspModel::getPointCloud() const
{
  throw std::runtime_error("PCLGraspModel::getPointCloud() is unsupported.");
}

void PCLGraspModel::setPointCloud(const sensor_msgs::PointCloud2 &point_cloud)
{
  // copy the point cloud
  point_cloud_metrics::rosPointCloud2ToPCLPointCloud(point_cloud, pc_);
  // compute RGB information
  point_cloud_metrics::calculateAvgColors(pc_, avg_r_, avg_g_, avg_b_);
}

double PCLGraspModel::getAverageRed() const
{
  return avg_r_;
}

double PCLGraspModel::getAverageGreen() const
{
  return avg_g_;
}

double PCLGraspModel::getAverageBlue() const
{
  return avg_b_;
}

graspdb::GraspModel PCLGraspModel::toGraspModel() const
{
  // convert the point cloud to a ROS message
  sensor_msgs::PointCloud2 msg;
  point_cloud_metrics::pclPointCloudToROSPointCloud2(this->getPCLPointCloud(), msg);

  // convert the basic values
  graspdb::GraspModel grasp_model(this->getID(), this->getObjectName(), this->getGrasps(), msg, this->getCreated());
  return grasp_model;
}
