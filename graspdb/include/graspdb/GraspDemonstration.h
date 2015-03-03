#ifndef RAIL_GRASPDB_GRASP_DEMONSTRATION_H_
#define RAIL_GRASPDB_GRASP_DEMONSTRATION_H_

#include <string>
#include <graspdb/Pose.h>
#include <stdint.h>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

class GraspDemonstration
{
public:
  static const uint32_t UNSET_ID = 0;
  static const time_t UNSET_TIME = 0;

  GraspDemonstration(const uint32_t id, const std::string object_name, const Pose grasp_pose, const uint8_t *point_cloud, const size_t point_cloud_size, const time_t created);

  GraspDemonstration(const std::string object_name, const Pose grasp_pose, const uint8_t *point_cloud, const size_t point_cloud_size);

  ~GraspDemonstration();

  uint32_t getID() const;

  void setID(const uint32_t id);

  const std::string &getObjectName() const;

  void setObjectName(const std::string object_name);

  const Pose &getGraspPose() const;

  void setGraspPose(const Pose grasp_pose);

  uint8_t *getPointCloud() const;

  void setPointCloud(const uint8_t *point_cloud, const size_t point_cloud_size);

  size_t getPointCloudSize() const;

  time_t getCreated() const;

  void setCreated(const time_t created);

private:
  void copyPointCloudBuffer(const uint8_t *point_cloud, const size_t point_cloud_size, const bool clean = false);

  uint32_t id_;
  std::string object_name_;
  Pose grasp_pose_;
  uint8_t *point_cloud_;
  size_t point_cloud_size_;
  time_t created_;
};

}
}
}

#endif
