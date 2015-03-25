#ifndef PC_RECOGNIZER_H_
#define PC_RECOGNIZER_H_

//ROS
#include <graspdb/graspdb.h>
#include <rail_recognition/Model.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//C++
#include <boost/thread/thread.hpp>
#include <stdlib.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

//Point Cloud Filtering Constants
#define RADIUS .01
#define NUM_NEIGHBORS 23
#define DST_THRESHOLD .00075

//Recognition Constants
#define ALPHA .5

namespace rail
{
namespace pick_and_place
{

class PCRecognizer
{
public:

/**
  * Constructor
  */
  PCRecognizer();

  bool recognizeObject(rail_manipulation_msgs::SegmentedObject *object, std::vector<graspdb::GraspModel> candidates);

private:
  float xTrans;
  float yTrans;
  float zTrans;

  //tf
  tf::TransformListener tfListener;
  tf::TransformBroadcaster tfBroadcaster;

  /**
  * Determine a score for the registration of two point clouds
  * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
  * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
  * @return score representing the success of the registration, calculated as a weighted combination of color and shape metrics
  */
  float scoreRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);

  /**
  * Register two point clouds together using an iterative closest point algorithm
  * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
  * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
  * @param baseGrasps list of grasps associated with the base point cloud
  * @param targetGrasps list of grasps associated with the target point cloud
  * @param resultGrasps pointer to a list where the combined grasps will be stored
  * @return resulting point cloud from the registration process
  */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr,
      std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> baseGrasps,
      std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> targetGrasps,
      std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> *resultGrasps);

  /**
  * Calculate a metric for how successful the registration was based on distance error
  * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
  * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
  * @return a score representing the success of the registration
  */
  float calculateRegistrationMetricDstError(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);

  /**
  * Calculate a metric for how successful the registration was based on overlap
  * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
  * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
  * @return a score representing the success of the registration
  */
  float calculateRegistrationMetricOverlap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr, float dstThreshold);

  /**
  * Calculate a metric for how successful the registration was based on range of colors in each point cloud
  * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
  * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
  * @return a score representing the success of the registration
  */
  float calculateRegistrationMetricColorRange(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);

  /**
  * Calculate a metric for how successful the registration was based on the respective spatial lengths of each point cloud
  * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
  * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
  * @return a score representing the success of the registration
  */
  float calculateRegistrationMetricDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);

  /**
  * Filters point cloud outliers if they have less neighbors than the neighbor threshold within a given radius
  * @param cloudPtr pointer to the point cloud to be filtered
  * @param radius the radius to search within for neighbors
  * @param numNeighborThreshold minimum number of neighbors required to keep a point
  */
  void filterCloudOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, double radius, int numNeighborThreshold);

  /**
  * Removes extra points that are within a given threshold of other points to keep the point cloud size manageable
  * @param cloudPtr pointer to the point cloud to be filtered
  * @param dstThreshold the minimum distance between neighboring points at which to keep a point
  */
  void filterRedundentPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, double dstThreshold);

  void translateToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> *grasps);

};

} //end namespace pick_and_place
} //end namespace rail

#endif
