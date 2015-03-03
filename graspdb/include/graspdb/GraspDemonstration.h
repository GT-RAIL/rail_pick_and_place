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

  GraspDemonstration(uint32_t id, std::string object_name, Pose grasp_pose, uint8_t *point_cloud, size_t point_cloud_size, time_t created);

  GraspDemonstration(std::string object_name, Pose grasp_pose, uint8_t *point_cloud, size_t point_cloud_size);

  ~GraspDemonstration();

  uint32_t getID() const;

  void setID(uint32_t id);

  std::string &getObjectName();

  void setObjectName(std::string object_name);

  Pose &getGraspPose();

  void setGraspPose(Pose grasp_pose);

  uint8_t *getPointCloud() const;

  void setPointCloud(uint8_t *point_cloud, size_t point_cloud_size);

  size_t getPointCloudSize() const;

  time_t getCreated() const;

  void setCreated(time_t created);

private:
  void copyPointCloudBuffer(uint8_t *point_cloud, size_t point_cloud_size, bool clean = false);

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
