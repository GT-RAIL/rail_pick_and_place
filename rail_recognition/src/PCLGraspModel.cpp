// RAIL Recognition
#include "rail_recognition/PCLGraspModel.h"
#include "rail_recognition/PointCloudMetrics.h"

using namespace std;
using namespace rail::pick_and_place;

PCLGraspModel::PCLGraspModel(const graspdb::GraspModel &grasp_model)
    : graspdb::GraspModel(grasp_model),
      pc_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  // copy the point cloud if it exists
  if (grasp_model.getPointCloud().data.size() > 0)
  {
    this->setPointCloud(grasp_model.getPointCloud());
  } else
  {
    // simply store the header information
    pc_->header.frame_id = grasp_model.getPointCloud().header.frame_id;
  }
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
