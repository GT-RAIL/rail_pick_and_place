#include <pcl/common/transforms.h>
#include <rail_recognition/PointCloudMetrics.h>

using namespace std;
using namespace rail::pick_and_place;

PointCloudMetrics::PointCloudMetrics()
{
}

void PointCloudMetrics::transformToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
    const geometry_msgs::Point &centroid) const
{
  // transformation matrix
  Eigen::Matrix4f tf;
  tf << 1, 0, 0, -centroid.x,
      0, 1, 0, -centroid.y,
      0, 0, 1, -centroid.z,
      0, 0, 0, 1;

  // transform the point cloud
  pcl::transformPointCloud(*pc, *pc, tf);
}

void PointCloudMetrics::transformToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
    vector<rail_pick_and_place_msgs::GraspWithSuccessRate> &grasps, const geometry_msgs::Point &centroid) const
{
  // start with the point cloud
  this->transformToOrigin(pc, centroid);

  // transform each grasp
  //for (size_t)
}