#ifndef RAIL_PICK_AND_PLACE_POINT_CLOUD_METRICS_H_
#define RAIL_PICK_AND_PLACE_POINT_CLOUD_METRICS_H_

// ROS
#include <geometry_msgs/Point.h>
#include <rail_pick_and_place_msgs/GraspWithSuccessRate.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// C++ Standard Library
#include <vector>

namespace rail
{
namespace pick_and_place
{

class PointCloudMetrics
{
public:
  PointCloudMetrics();

  void transformToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const geometry_msgs::Point &centroid) const;

  void transformToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
      std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> &grasps, const geometry_msgs::Point &centroid) const;
};

}
}

#endif
