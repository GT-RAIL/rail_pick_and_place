#include <graspdb/GraspDemonstration.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

GraspDemonstration::GraspDemonstration(uint32_t id, string object_name, Pose grasp_pose, uint8_t *point_cloud, size_t point_cloud_size, time_t created)
    : object_name_(object_name), grasp_pose_(grasp_pose)
{
  id_ = id;
  created_ = created;
  // copy the serialized point cloud data
  this->copyPointCloudBuffer(point_cloud, point_cloud_size);
}

GraspDemonstration::GraspDemonstration(string object_name, Pose grasp_pose, uint8_t *point_cloud, size_t point_cloud_size)
    : object_name_(object_name), grasp_pose_(grasp_pose)
{
  // default values
  id_ = UNSET_ID;
  created_ = UNSET_TIME;
  // copy the serialized point cloud data
  this->copyPointCloudBuffer(point_cloud, point_cloud_size);
}

GraspDemonstration::~GraspDemonstration()
{
  // check if we need to clean old data
  if (point_cloud_size_ > 0)
  {
    delete point_cloud_;
  }
}

uint32_t GraspDemonstration::getID() const
{
  return id_;
}

void GraspDemonstration::setID(uint32_t id)
{
  id_ = id;
}

std::string &GraspDemonstration::getObjectName()
{
  return object_name_;
}

void GraspDemonstration::setObjectName(std::string object_name)
{
  object_name_ = object_name;
}

Pose &GraspDemonstration::getGraspPose()
{
  return grasp_pose_;
}

void GraspDemonstration::setGraspPose(Pose grasp_pose)
{
  grasp_pose_ = grasp_pose;
}

uint8_t *GraspDemonstration::getPointCloud() const
{
  return point_cloud_;
}

void GraspDemonstration::setPointCloud(uint8_t *point_cloud, size_t point_cloud_size)
{
  // perform the copy and clear the old data
  this->copyPointCloudBuffer(point_cloud, point_cloud_size, true);
}

size_t GraspDemonstration::getPointCloudSize() const
{
  return point_cloud_size_;
}

time_t GraspDemonstration::getCreated() const
{
  return created_;
}

void GraspDemonstration::setCreated(time_t created)
{
  created_ = created;
}

void GraspDemonstration::copyPointCloudBuffer(uint8_t *point_cloud, size_t point_cloud_size, bool clean)
{
  // check if we need to clean old data
  if (clean && point_cloud_size_ > 0)
  {
    delete point_cloud_;
  }

  // perform the copy
  point_cloud_size_ = point_cloud_size;
  if (point_cloud_size > 0)
  {
    point_cloud_ = new uint8_t[point_cloud_size_];
    std::copy(point_cloud_, point_cloud_ + point_cloud_size_, point_cloud_);
  } else
  {
    point_cloud_ = NULL;
  }
}