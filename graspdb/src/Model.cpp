/*!
 * \file Model.cpp
 * \brief Trained grasp model information.
 *
 * A grasp model contains a 3D point cloud model made up of several segmented point cloud segments from various grasp
 * demonstrations, an object name (may be non-unique), an array of grasp poses for the model, a unique identifier,
 * counters for trials/success rates for the grasps, and a timestamp.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

#include <graspdb/Model.h>
#include <lber.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

Model::Model(const uint32_t id, const string &object_name, const sensor_msgs::PointCloud2 &point_cloud,
    const time_t created) : object_name_(object_name), point_cloud_(point_cloud)
{
  id_ = id;
  created_ = created;
}

Model::Model(const string &object_name, const sensor_msgs::PointCloud2 &point_cloud)
    : object_name_(object_name), point_cloud_(point_cloud)
{
  // default values
  id_ = UNSET_ID;
  created_ = UNSET_TIME;
}

Model::Model(const rail_pick_and_place_msgs::Model &m)
    : object_name_(m.object_name), point_cloud_(m.point_cloud)
{
  // set the ID and timestamp
  id_ = m.id;
  created_ = m.created.sec;

  // use the smallest size to not create an index out of bounds issue
  size_t num_poses = m.grasp_poses.size();
  size_t num_frame_ids = m.grasp_frame_ids.size();
  size_t num_successes = m.successes.size();
  size_t num_attempts = m.attempts.size();
  size_t size = min(num_poses, min(num_frame_ids, min(num_successes, num_attempts)));
  // copy over the values
  for (size_t i = 0; i < size; i++)
  {
    // create a graspdb Pose object
   //TODO Pose p(m.grasp_frame_ids[i], m.grasp_frame_ids[i], m.grasp_poses[i]);
    // store the pose
   //TODO this->addGraspPose(p, m.grasp_frame_ids[i], m.successes[i], m.attempts[i]);
  }
}

uint32_t Model::getID() const
{
  return id_;
}

void Model::setID(const uint32_t id)
{
  id_ = id;
}

const string &Model::getObjectName() const
{
  return object_name_;
}

void Model::setObjectName(const string &object_name)
{
  object_name_ = object_name;
}

const vector<Pose> &Model::getGraspPoses() const
{
  return grasp_poses_;
}

size_t Model::getNumGraspPoses() const
{
  return grasp_poses_.size();
}

const Pose &Model::getGraspPose(const size_t i) const
{
  return grasp_poses_[i];
}

void Model::addGraspPose(const Pose &grasp_pose, const string &grasp_frame_id)
{
  // no attempt/success information
  this->addGraspPose(grasp_pose, grasp_frame_id, 0, 0);
}

void Model::addGraspPose(const Pose &grasp_pose, const string &grasp_frame_id, const uint32_t successes,
    const uint32_t attempts)
{
  // add each value
  grasp_poses_.push_back(grasp_pose);
  grasp_frame_ids_.push_back(grasp_frame_id);
  successes_.push_back(successes);
  attempts_.push_back(attempts);
}

void Model::removeGraspPose(const size_t i)
{
  // check the size
  if (i < grasp_poses_.size() && i < grasp_frame_ids_.size() && i < successes_.size() && i < attempts_.size())
  {
    // remove from each
    grasp_poses_.erase(grasp_poses_.begin() + i);
    grasp_frame_ids_.erase(grasp_frame_ids_.begin() + i);
    successes_.erase(successes_.begin() + i);
    attempts_.erase(attempts_.begin() + i);
  }
}


const string &Model::getGraspFrameID(size_t i) const
{
  return grasp_frame_ids_[i];
}

const vector<string> &Model::getGraspFrameIDs() const
{
  return grasp_frame_ids_;
}

const vector<uint32_t> &Model::getSuccesses() const
{
  return successes_;
}

uint32_t Model::getSuccesses(size_t i) const
{
  if (i < successes_.size())
  {
    return successes_[i];
  } else
  {
    return 0;
  }
}

const vector<uint32_t> &Model::getAttempts() const
{
  return attempts_;
}

uint32_t Model::getAttempts(size_t i) const
{
  if (i < attempts_.size())
  {
    return attempts_[i];
  } else
  {
    return 0;
  }
}

vector<double> Model::getSuccessRates() const
{
  // create the vector
  size_t size = this->getNumGraspPoses();
  vector<double> rates(size);

  // fill it with each success rate
  for (size_t i = 0; i < size; i++)
  {
    rates.push_back(this->getSuccessRate(i));
  }

  return rates;
}

double Model::getSuccessRate(size_t i) const
{
  if (i < successes_.size() && i < attempts_.size())
  {
    // check for a divide by zero
    uint32_t successes = this->getSuccesses(i);
    uint32_t attempts = this->getAttempts(i);
    return (attempts == 0) ? 0 : ((double) successes) / ((double) attempts);
  } else
  {
    // invalid index
    return -1;
  }
}

size_t Model::getBestGraspIndex() const
{
  size_t index = 0;
  double best = 0;

  // check each rate
  vector<double> success_rates = this->getSuccessRates();
  for (size_t i = 0; i < success_rates.size(); i++)
  {
    // check for a better value
    if (success_rates[i] > best)
    {
      best = success_rates[i];
      index = i;
    }
  }

  return index;
}

const sensor_msgs::PointCloud2 &Model::getPointCloud() const
{
  return point_cloud_;
}

void Model::setPointCloud(const sensor_msgs::PointCloud2 &point_cloud)
{
  point_cloud_ = point_cloud;
}

time_t Model::getCreated() const
{
  return created_;
}

void Model::setCreated(const time_t created)
{
  created_ = created;
}

rail_pick_and_place_msgs::Model Model::toROSModelMessage() const
{
  rail_pick_and_place_msgs::Model m;
  m.id = id_;
  m.object_name = object_name_;

  // copy over vector data
  for (size_t i = 0; i < this->getNumGraspPoses(); i++)
  {
    m.grasp_poses.push_back(grasp_poses_[i].toROSPoseMessage());
    m.grasp_frame_ids.push_back(grasp_frame_ids_[i]);
    m.successes.push_back(successes_[i]);
    m.attempts.push_back(attempts_[i]);
  }

  m.point_cloud = point_cloud_;
  m.created.nsec = 0;
  m.created.sec = created_;
  return m;
}
