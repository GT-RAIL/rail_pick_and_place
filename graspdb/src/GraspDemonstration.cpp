/*!
 * \file GraspDemonstration.cpp
 * \brief A grasp demonstration database entry.
 *
 * A grasp demonstration contains information about a single grasp demonstration in the grasp database. This
 * contains information about the grasp pose, object name, and serialized segmented point cloud.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

#include <graspdb/GraspDemonstration.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

GraspDemonstration::GraspDemonstration(const uint32_t id, const string object_name, const Pose grasp_pose,
    const uint8_t *point_cloud, const uint32_t point_cloud_size, const time_t created) : object_name_(object_name),
                                                                                         grasp_pose_(grasp_pose)
{
  id_ = id;
  created_ = created;
  // copy the serialized point cloud data
  this->copyPointCloudBuffer(point_cloud, point_cloud_size);
}

GraspDemonstration::GraspDemonstration(const string object_name, const Pose grasp_pose, const uint8_t *point_cloud,
    const uint32_t point_cloud_size) : object_name_(object_name), grasp_pose_(grasp_pose)
{
  // default values
  id_ = UNSET_ID;
  created_ = UNSET_TIME;
  // copy the serialized point cloud data
  this->copyPointCloudBuffer(point_cloud, point_cloud_size);
}

GraspDemonstration::GraspDemonstration(const string object_name, const string grasp_pose_fixed_frame_id,
    const string grasp_pose_grasp_frame_id, const geometry_msgs::Pose &grasp_pose,
    const sensor_msgs::PointCloud2 &point_cloud) : object_name_(object_name),
                                                   grasp_pose_(
                                                       grasp_pose_grasp_frame_id, grasp_pose_fixed_frame_id, grasp_pose
                                                   )
{
  // serialize and copy the point cloud data
  this->copyPointCloudBuffer(point_cloud);
}

GraspDemonstration::GraspDemonstration(const string object_name, const string grasp_pose_fixed_frame_id,
    const string grasp_pose_grasp_frame_id, const geometry_msgs::Transform &grasp_pose,
    const sensor_msgs::PointCloud2 &point_cloud) : object_name_(object_name),
                                                   grasp_pose_(
                                                       grasp_pose_grasp_frame_id, grasp_pose_fixed_frame_id, grasp_pose
                                                   )
{
  // serialize and copy the point cloud data
  this->copyPointCloudBuffer(point_cloud);
}

GraspDemonstration::GraspDemonstration(const rail_pick_and_place_msgs::GraspDemonstration &gd)
    : object_name_(gd.object_name),
      grasp_pose_(gd.grasp_pose_fixed_frame_id, gd.grasp_pose_grasp_frame_id, gd.grasp_pose)
{
  // set the ID and timestamp
  id_ = gd.id;
  created_ = gd.created.sec;
  // serialize and copy the point cloud data
  this->copyPointCloudBuffer(gd.point_cloud);
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

const uint8_t *GraspDemonstration::getPointCloud() const
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

void GraspDemonstration::copyPointCloudBuffer(const uint8_t *point_cloud, const uint32_t point_cloud_size,
    const bool clean)
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
