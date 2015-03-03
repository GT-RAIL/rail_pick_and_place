#include <graspdb/GraspDemonstration.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

GraspDemonstration::GraspDemonstration(const uint32_t id, const string object_name, const Pose grasp_pose, const uint8_t *point_cloud, const uint32_t point_cloud_size, const time_t created)
    : object_name_(object_name), grasp_pose_(grasp_pose)
{
  id_ = id;
  created_ = created;
  // copy the serialized point cloud data
  this->copyPointCloudBuffer(point_cloud, point_cloud_size);
}

GraspDemonstration::GraspDemonstration(const string object_name, const Pose grasp_pose, const uint8_t *point_cloud, const uint32_t point_cloud_size)
    : object_name_(object_name), grasp_pose_(grasp_pose)
{
  // default values
  id_ = UNSET_ID;
  created_ = UNSET_TIME;
  // copy the serialized point cloud data
  this->copyPointCloudBuffer(point_cloud, point_cloud_size);
}

GraspDemonstration::GraspDemonstration(const string object_name, const string grasp_pose_frame_id, const geometry_msgs::Pose &grasp_pose, const sensor_msgs::PointCloud2 &point_cloud)
    : object_name_(object_name), grasp_pose_(grasp_pose_frame_id, grasp_pose)
{
  // serialize and copy the point cloud data
  this->copyPointCloudBuffer(point_cloud);
}

GraspDemonstration::GraspDemonstration(const string object_name, const string grasp_pose_frame_id, const geometry_msgs::Transform &grasp_pose, const sensor_msgs::PointCloud2 &point_cloud)
    : object_name_(object_name), grasp_pose_(grasp_pose_frame_id, grasp_pose)
{
  // serialize and copy the point cloud data
  this->copyPointCloudBuffer(point_cloud);
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

void GraspDemonstration::setID(const uint32_t id)
{
  id_ = id;
}

const string &GraspDemonstration::getObjectName() const
{
  return object_name_;
}

void GraspDemonstration::setObjectName(const string object_name)
{
  object_name_ = object_name;
}

const Pose &GraspDemonstration::getGraspPose() const
{
  return grasp_pose_;
}

void GraspDemonstration::setGraspPose(const Pose grasp_pose)
{
  grasp_pose_ = grasp_pose;
}

uint8_t *GraspDemonstration::getPointCloud() const
{
  return point_cloud_;
}

void GraspDemonstration::setPointCloud(const uint8_t *point_cloud, const uint32_t point_cloud_size)
{
  // perform the copy and clear the old data
  this->copyPointCloudBuffer(point_cloud, point_cloud_size, true);
}

void GraspDemonstration::setPointCloud(const sensor_msgs::PointCloud2 &point_cloud)
{
  // perform the copy and clear the old data
  this->copyPointCloudBuffer(point_cloud, true);
}

uint32_t GraspDemonstration::getPointCloudSize() const
{
  return point_cloud_size_;
}

time_t GraspDemonstration::getCreated() const
{
  return created_;
}

void GraspDemonstration::setCreated(const time_t created)
{
  created_ = created;
}

void GraspDemonstration::copyPointCloudBuffer(const uint8_t *point_cloud, const uint32_t point_cloud_size, const bool clean)
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

void GraspDemonstration::copyPointCloudBuffer(const sensor_msgs::PointCloud2 &point_cloud, const bool clean)
{
  // check if we need to clean old data
  if (clean && point_cloud_size_ > 0)
  {
    delete point_cloud_;
  }

  // determine the size for the buffer
  point_cloud_size_ = ros::serialization::serializationLength(point_cloud);
  point_cloud_ = new uint8_t[point_cloud_size_];
  // serilize the message
  ros::serialization::OStream stream(point_cloud_, point_cloud_size_);
  ros::serialization::serialize(stream, point_cloud);
}
