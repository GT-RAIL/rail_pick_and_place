/*!
 * \file Grasp.cpp
 * \brief A grasp database entity.
 *
 * A grasp contains information about a single grasp in the grasp database. This contains information about the grasp
 * pose, end effector frame identifier, number of recorded successful grasps, number of recorded attempted grasps,
 * and associated model ID.  A valid database  entity has an ID and created timestamp. This class is useful for
 * internal data management within the graspdb library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

// graspdb
#include "graspdb/Grasp.h"

using namespace std;
using namespace rail::pick_and_place::graspdb;

Grasp::Grasp(const uint32_t id, const uint32_t grasp_model_id, const Pose &grasp_pose, const string &eef_frame_id,
    const uint32_t successes, const uint32_t attempts, const time_t created)
    : Entity(id, created),
      grasp_pose_(grasp_pose), eef_frame_id_(eef_frame_id)
{
  grasp_model_id_ = grasp_model_id;
  successes_ = successes;
  attempts_ = attempts;
}

Grasp::Grasp(const Pose &grasp_pose, const uint32_t grasp_model_id, const string &eef_frame_id,
    const uint32_t successes, const uint32_t attempts)
    : grasp_pose_(grasp_pose), eef_frame_id_(eef_frame_id)
{
  grasp_model_id_ = grasp_model_id;
  successes_ = successes;
  attempts_ = attempts;
}

Grasp::Grasp(const rail_pick_and_place_msgs::GraspWithSuccessRate &g, const uint32_t grasp_model_id)
    : Entity(g.id, g.created.sec),
      grasp_pose_(g.grasp_pose), eef_frame_id_(g.eef_frame_id)
{
  grasp_model_id_ = grasp_model_id;
  successes_ = 0;
  attempts_ = 0;
}

uint32_t Grasp::getGraspModelID() const
{
  return grasp_model_id_;
}

void Grasp::setGraspModelID(const uint32_t grasp_model_id)
{
  grasp_model_id_ = grasp_model_id;
}

const Pose &Grasp::getGraspPose() const
{
  return grasp_pose_;
}

Pose &Grasp::getGraspPose()
{
  return grasp_pose_;
}

void Grasp::setGraspPose(const Pose &grasp_pose)
{
  grasp_pose_ = grasp_pose;
}

const string &Grasp::getEefFrameID() const
{
  return eef_frame_id_;
}

void Grasp::setEefFrameID(const string &eef_frame_id)
{
  eef_frame_id_ = eef_frame_id;
}

uint32_t Grasp::getSuccesses() const
{
  return successes_;
}

void Grasp::setSuccesses(const uint32_t successes)
{
  successes_ = successes;
}

uint32_t Grasp::getAttempts() const
{
  return attempts_;
}

void Grasp::setAttempts(const uint32_t attempts)
{
  attempts_ = attempts;
}

double Grasp::getSuccessRate() const
{
  // check for a 0 attempt
  return (attempts_ == 0) ? 0 : ((double) successes_) / ((double) attempts_);
}


rail_pick_and_place_msgs::GraspWithSuccessRate Grasp::toROSGraspWithSuccessRateMessage() const
{
  rail_pick_and_place_msgs::GraspWithSuccessRate g;
  g.id = this->getID();
  g.grasp_pose = grasp_pose_.toROSPoseStampedMessage();
  g.eef_frame_id = eef_frame_id_;
  g.successes = successes_;
  g.attempts = attempts_;
  g.created.nsec = 0;
  g.created.sec = this->getCreated();
  return g;
}
