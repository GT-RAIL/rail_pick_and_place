/*!
 * \file GraspDemonstration.h
 * \brief A grasp demonstration database entity.
 *
 * A grasp demonstration contains information about a single grasp demonstration in the grasp database. This
 * contains information about the grasp pose, end effector frame identifier, object name, and serialized segmented
 * point cloud. A valid database entity has an ID and created timestamp. This class is useful for internal data
 * management within the graspdb library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

// graspdb
#include "graspdb/GraspDemonstration.h"

using namespace std;
using namespace rail::pick_and_place::graspdb;

GraspDemonstration::GraspDemonstration(const uint32_t id, const string &object_name, const Pose &grasp_pose,
                                       const string &eef_frame_id, const sensor_msgs::PointCloud2 &point_cloud,
                                       const sensor_msgs::Image &image, const time_t created)
    : Entity(id, created),
      object_name_(object_name), grasp_pose_(grasp_pose), eef_frame_id_(eef_frame_id), point_cloud_(point_cloud),
      image_(image)
{
}

GraspDemonstration::GraspDemonstration(const string &object_name, const Pose &grasp_pose, const string &eef_frame_id,
                                       const sensor_msgs::PointCloud2 &point_cloud, const sensor_msgs::Image &image)
    : object_name_(object_name), grasp_pose_(grasp_pose), eef_frame_id_(eef_frame_id), point_cloud_(point_cloud),
      image_(image)
{
}

GraspDemonstration::GraspDemonstration(const rail_pick_and_place_msgs::GraspDemonstration &gd)
    : Entity(gd.id, gd.created.sec),
      object_name_(gd.object_name), grasp_pose_(gd.grasp_pose), eef_frame_id_(gd.eef_frame_id),
      point_cloud_(gd.point_cloud), image_(gd.image)
{
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

Pose &GraspDemonstration::getGraspPose()
{
  return grasp_pose_;
}

void GraspDemonstration::setGraspPose(const Pose &grasp_pose)
{
  grasp_pose_ = grasp_pose;
}

const string &GraspDemonstration::getEefFrameID() const
{
  return eef_frame_id_;
}

void GraspDemonstration::setEefFrameID(const string &eef_frame_id)
{
  eef_frame_id_ = eef_frame_id;
}

const sensor_msgs::PointCloud2 &GraspDemonstration::getPointCloud() const
{
  return point_cloud_;
}

sensor_msgs::PointCloud2 &GraspDemonstration::getPointCloud()
{
  return point_cloud_;
}

void GraspDemonstration::setPointCloud(const sensor_msgs::PointCloud2 &point_cloud)
{
  point_cloud_ = point_cloud;
}

const sensor_msgs::Image &GraspDemonstration::getImage() const
{
  return image_;
}

sensor_msgs::Image &GraspDemonstration::getImage()
{
  return image_;
}

void GraspDemonstration::setImage(const sensor_msgs::Image &image)
{
  image_ = image;
}

rail_pick_and_place_msgs::GraspDemonstration GraspDemonstration::toROSGraspDemonstrationMessage() const
{
  rail_pick_and_place_msgs::GraspDemonstration gd;
  gd.id = this->getID();
  gd.object_name = object_name_;
  gd.grasp_pose = grasp_pose_.toROSPoseStampedMessage();
  gd.eef_frame_id = eef_frame_id_;
  gd.point_cloud = point_cloud_;
  gd.image = image_;
  gd.created.nsec = 0;
  gd.created.sec = this->getCreated();
  return gd;
}
