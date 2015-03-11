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

GraspDemonstration::GraspDemonstration(const uint32_t id, const string &object_name, const Pose &grasp_pose,
    const sensor_msgs::PointCloud2 &point_cloud, const time_t created)
    : object_name_(object_name), grasp_pose_(grasp_pose), point_cloud_(point_cloud)
{
  id_ = id;
  created_ = created;
}

GraspDemonstration::GraspDemonstration(const string &object_name, const Pose &grasp_pose,
    const sensor_msgs::PointCloud2 &point_cloud)
    : object_name_(object_name), grasp_pose_(grasp_pose), point_cloud_(point_cloud)
{
  // default values
  id_ = UNSET_ID;
  created_ = UNSET_TIME;
}

GraspDemonstration::GraspDemonstration(const string &object_name, const string &grasp_pose_fixed_frame_id,
    const string &grasp_pose_grasp_frame_id, const geometry_msgs::Pose &grasp_pose,
    const sensor_msgs::PointCloud2 &point_cloud)
    : object_name_(object_name), grasp_pose_(grasp_pose_fixed_frame_id, grasp_pose_grasp_frame_id, grasp_pose),
      point_cloud_(point_cloud)
{
}

GraspDemonstration::GraspDemonstration(const string &object_name, const string &grasp_pose_fixed_frame_id,
    const string &grasp_pose_grasp_frame_id, const geometry_msgs::Transform &grasp_pose,
    const sensor_msgs::PointCloud2 &point_cloud)
    : object_name_(object_name), grasp_pose_(grasp_pose_fixed_frame_id, grasp_pose_grasp_frame_id, grasp_pose),
      point_cloud_(point_cloud)
{
}

GraspDemonstration::GraspDemonstration(const rail_pick_and_place_msgs::GraspDemonstration &gd)
    : object_name_(gd.object_name), point_cloud_(gd.point_cloud),
      grasp_pose_(gd.grasp_pose_fixed_frame_id, gd.grasp_pose_grasp_frame_id, gd.grasp_pose)
{
  // set the ID and timestamp
  id_ = gd.id;
  created_ = gd.created.sec;
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

void GraspDemonstration::setObjectName(const string &object_name)
{
  object_name_ = object_name;
}

const Pose &GraspDemonstration::getGraspPose() const
{
  return grasp_pose_;
}

void GraspDemonstration::setGraspPose(const Pose &grasp_pose)
{
  grasp_pose_ = grasp_pose;
}

const sensor_msgs::PointCloud2 &GraspDemonstration::getPointCloud() const
{
  return point_cloud_;
}

void GraspDemonstration::setPointCloud(const sensor_msgs::PointCloud2 &point_cloud)
{
  point_cloud_ = point_cloud;
}

time_t GraspDemonstration::getCreated() const
{
  return created_;
}

void GraspDemonstration::setCreated(const time_t created)
{
  created_ = created;
}

rail_pick_and_place_msgs::GraspDemonstration GraspDemonstration::toROSPGraspDemonstrationMessage() const
{
  rail_pick_and_place_msgs::GraspDemonstration gd;
  gd.id = id_;
  gd.object_name = object_name_;
  gd.grasp_pose_fixed_frame_id = grasp_pose_.getFixedFrameID();
  gd.grasp_pose_grasp_frame_id = grasp_pose_.getGraspFrameID();
  gd.grasp_pose = grasp_pose_.toROSPoseMessage();
  gd.point_cloud = point_cloud_;
  gd.created.nsec = 0;
  gd.created.sec = created_;
  return gd;
}
