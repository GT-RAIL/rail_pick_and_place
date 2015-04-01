#ifndef RAIL_PICK_AND_PLACE_POINT_CLOUD_RECOGNIZER_H_
#define RAIL_PICK_AND_PLACE_POINT_CLOUD_RECOGNIZER_H_

// ROS
#include <graspdb/graspdb.h>
#include <rail_recognition/Model.h>
#include <sensor_msgs/PointCloud.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Pick and Place
#include "PointCloudMetrics.h"

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
  * Filters point cloud outliers if they have less neighbors than the neighbor threshold within a given radius
  * @param cloudPtr pointer to the point cloud to be filtered
  */
  void filterPointCloudOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr) const;

  void translateToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, const geometry_msgs::Point &centroid) const;

  /**
  * Determine a score for the registration of two point clouds
  * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
  * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
  * @return score representing the success of the registration, calculated as a weighted combination of color and shape metrics
  */
  double scoreRegistration(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr candidate,
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr object, Eigen::Matrix4f &icp_tf, bool &icp_swapped) const;

  /**
  * Calculate a metric for how successful the registration was based on distance error
  * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
  * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
  * @return a score representing the success of the registration
  */
  double calculateRegistrationMetricDistanceError(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr base,
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target) const;

  /**
  * Calculate a metric for how successful the registration was based on overlap
  * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
  * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
  * @return a score representing the success of the registration
  */
  double calculateRegistrationMetricOverlap(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr base,
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target) const;

  void transformGrasps(const Eigen::Matrix4f &icp_transform, const bool swapped,
      const geometry_msgs::Point &centroid,
      const std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> &candidate_grasps,
      std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> &grasps) const;

  PointCloudMetrics metrics_;
};

}
}

#endif
