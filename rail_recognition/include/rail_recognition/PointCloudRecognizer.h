#ifndef RAIL_PICK_AND_PLACE_POINT_CLOUD_RECOGNIZER_H_
#define RAIL_PICK_AND_PLACE_POINT_CLOUD_RECOGNIZER_H_

// ROS
#include <graspdb/graspdb.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <rail_manipulation_msgs/SegmentedObject.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rail
{
namespace pick_and_place
{

class PointCloudRecognizer
{
public:
  /*! The radius to search within for neighbors during pre-processing. */
  static const double FILTER_SEARCH_RADIUS = 0.01;
  /*! The minimum number of neighbors required to keep a point during pre-processing. */
  static const int FILTER_MIN_NUM_NEIGHBORS = 23;
  /*! The weighting constant for the match score metric. */
  static const double ALPHA = 0.5;
  /*! The radius to search within for neighbors during the overlap metric search. */
  static const double OVERLAP_SEARCH_RADIUS = 0.005;
  /*! The confidence threshold for a recognition score. */
  static const double SCORE_CONFIDENCE_THRESHOLD = 0.8;

  PointCloudRecognizer();

  bool recognizeObject(rail_manipulation_msgs::SegmentedObject &object,
      const std::vector<graspdb::GraspModel> &candidates) const;

private:


  /**
  * Determine a score for the registration of two point clouds
  * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
  * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
  * @return score representing the success of the registration, calculated as a weighted combination of color and shape metrics
  */
  double scoreRegistration(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr candidate,
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr object, Eigen::Matrix4f &icp_tf, bool &icp_swapped) const;

  /**
  * Calculate a metric for how successful the registration was based on overlap
  * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
  * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
  * @return a score representing the success of the registration
  */
  double calculateRegistrationMetricOverlap(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr base,
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target) const;

  std::vector<graspdb::Grasp> computeGraspList(const Eigen::Matrix4f &icp_transform, const bool swapped,
      const geometry_msgs::Point &centroid, const std::vector<graspdb::Grasp> &candidate_grasps) const;
};

}
}

#endif
