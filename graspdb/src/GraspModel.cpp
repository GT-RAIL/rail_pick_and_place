/*!
 * \file GraspModel.h
 * \brief Trained grasp model information.
 *
 * A grasp model contains a 3D point cloud model made up of several segmented point cloud segments from various grasp
 * demonstrations, an object name (may be non-unique), an array of grasps for the model, and a unique identifier.
 * This class is useful for internal data management within the graspdb library. Convenience functions are added for
 * use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 12, 2015
 */

// graspdb
#include "graspdb/GraspModel.h"

// ROS
#include <ros/ros.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

GraspModel::GraspModel(const uint32_t id, const string &object_name, const vector<Grasp> &grasps,
    const sensor_msgs::PointCloud2 &point_cloud, const time_t created)
    : Entity(id, created),
      object_name_(object_name), grasps_(grasps), point_cloud_(point_cloud)
{
}

GraspModel::GraspModel(const string &object_name, const vector<Grasp> &grasps,
    const sensor_msgs::PointCloud2 &point_cloud)
    : object_name_(object_name), grasps_(grasps), point_cloud_(point_cloud)
{
}

GraspModel::GraspModel(const rail_pick_and_place_msgs::GraspModel &gm)
    : Entity(gm.id, gm.created.sec),
      object_name_(gm.object_name), point_cloud_(gm.point_cloud)
{
  // copy over the grasp values
  for (size_t i = 0; i < gm.grasps.size(); i++)
  {
    Grasp grasp(gm.grasps[i], gm.id);
    grasps_.push_back(grasp);
  }
}

const string &GraspModel::getObjectName() const
{
  return object_name_;
}

void GraspModel::setObjectName(const string &object_name)
{
  object_name_ = object_name;
}

const vector<Grasp> &GraspModel::getGrasps() const
{
  return grasps_;
}

vector<Grasp> &GraspModel::getGrasps()
{
  return grasps_;
}

size_t GraspModel::getNumGrasps() const
{
  return grasps_.size();
}

const Grasp &GraspModel::getGrasp(const size_t index) const
{
  // check the index value first
  if (index < grasps_.size())
  {
    return grasps_[index];
  } else
  {
    throw std::out_of_range("GraspModel::getGrasp : Grasp index does not exist.");
  }
}

Grasp &GraspModel::getGrasp(const size_t index)
{
  // check the index value first
  if (index < grasps_.size())
  {
    return grasps_[index];
  } else
  {
    throw std::out_of_range("GraspModel::getGrasp : Grasp index does not exist.");
  }
}

void GraspModel::addGrasp(const Grasp &grasp)
{
  // verify the ID
  if (grasp.getGraspModelID() == this->getID())
  {
    grasps_.push_back(grasp);
  } else
  {
    ROS_WARN("GraspModel::addGrasp : Grasp Model ID mismatch. Grasp not added to model.");
  }
}

void GraspModel::removeGrasp(const size_t index)
{
  // check the index value first
  if (index < grasps_.size())
  {
    grasps_.erase(grasps_.begin() + index);
  } else
  {
    throw std::out_of_range("GraspModel::removeGraspPose : Grasp index does not exist.");
  }
}

const Grasp &GraspModel::getBestGrasp() const
{
  // check the index value first
  if (grasps_.size() > 0)
  {
    return grasps_[this->getBestGraspIndex()];
  } else
  {
    throw std::out_of_range("GraspModel::getBestGrasp : Grasp list is empty.");
  }
}

Grasp &GraspModel::getBestGrasp()
{
  // check the index value first
  if (grasps_.size() > 0)
  {
    return grasps_[this->getBestGraspIndex()];
  } else
  {
    throw std::out_of_range("GraspModel::getBestGrasp : Grasp list is empty.");
  }
}

size_t GraspModel::getBestGraspIndex() const
{
  // check the index value first
  if (grasps_.size() > 0)
  {
    size_t index = 0;
    double best = 0;

    // check each success rate
    for (size_t i = 0; i < grasps_.size(); i++)
    {
      // check for a better value
      if (grasps_[i].getSuccessRate() > best)
      {
        best = grasps_[i].getSuccessRate();
        index = i;
      }
    }

    return index;
  } else
  {
    throw std::out_of_range("GraspModel::getBestGraspIndex : Grasp list is empty.");
  }
}

double GraspModel::getBestSuccessRate() const
{
  // check the index value first
  if (grasps_.size() > 0)
  {
    // check each success rate
    double best = 0;
    for (size_t i = 0; i < grasps_.size(); i++)
    {
      // check for a better value
      if (grasps_[i].getSuccessRate() > best)
      {
        best = grasps_[i].getSuccessRate();
      }
    }
    return best;
  } else
  {
    throw std::out_of_range("GraspModel::getBestSuccessRate : Grasp list is empty.");
  }
}

const Grasp &GraspModel::getWorstGrasp() const
{
  // check the index value first
  if (grasps_.size() > 0)
  {
    return grasps_[this->getWorstGraspIndex()];
  } else
  {
    throw std::out_of_range("GraspModel::getWorstGrasp : Grasp list is empty.");
  }
}

Grasp &GraspModel::getWorstGrasp()
{
  // check the index value first
  if (grasps_.size() > 0)
  {
    return grasps_[this->getWorstGraspIndex()];
  } else
  {
    throw std::out_of_range("GraspModel::getWorstGrasp : Grasp list is empty.");
  }
}

size_t GraspModel::getWorstGraspIndex() const
{
  // check the index value first
  if (grasps_.size() > 0)
  {
    size_t index = 0;
    double worst = 1.0;

    // check each success rate
    for (size_t i = 0; i < grasps_.size(); i++)
    {
      // check for a better value
      if (grasps_[i].getSuccessRate() < worst)
      {
        worst = grasps_[i].getSuccessRate();
        index = i;
      }
    }

    return index;
  } else
  {
    throw std::out_of_range("GraspModel::getWorstGraspIndex : Grasp list is empty.");
  }
}

double GraspModel::getWorstSuccessRate() const
{
  // check the index value first
  if (grasps_.size() > 0)
  {
    // check each success rate
    double worst = 1.0;
    for (size_t i = 0; i < grasps_.size(); i++)
    {
      // check for a better value
      if (grasps_[i].getSuccessRate() < worst)
      {
        worst = grasps_[i].getSuccessRate();
      }
    }
    return worst;
  } else
  {
    throw std::out_of_range("GraspModel::getWorstSuccessRate : Grasp list is empty.");
  }
}

const sensor_msgs::PointCloud2 &GraspModel::getPointCloud() const
{
  return point_cloud_;
}

sensor_msgs::PointCloud2 &GraspModel::getPointCloud()
{
  return point_cloud_;
}

void GraspModel::setPointCloud(const sensor_msgs::PointCloud2 &point_cloud)
{
  point_cloud_ = point_cloud;
}

rail_pick_and_place_msgs::GraspModel GraspModel::toROSGraspModelMessage() const
{
  rail_pick_and_place_msgs::GraspModel gm;
  gm.id = this->getID();
  gm.object_name = object_name_;
  // copy over vector data
  for (size_t i = 0; i < grasps_.size(); i++)
  {
    gm.grasps.push_back(grasps_[i].toROSGraspWithSuccessRateMessage());
  }
  gm.point_cloud = point_cloud_;
  gm.created.nsec = 0;
  gm.created.sec = this->getCreated();
  return gm;
}
