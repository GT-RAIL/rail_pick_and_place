//ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseArray.h>
#include <graspdb/graspdb.h>
#include <rail_pick_and_place_msgs/GenerateModelsAction.h>
#include <rail_pick_and_place_msgs/GraspWithSuccessRate.h>
#include <rail_recognition/Model.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

// Pick and Place
#include "PointCloudMetrics.h"

//Point Cloud Filtering Constants
#define RADIUS .01
#define NUM_NEIGHBORS 23
#define DST_THRESHOLD .00075

namespace rail
{
namespace pick_and_place
{

class ModelGenerator
{
public:
  /*! If a topic should be created to display debug information such as model point clouds. */
  static const bool DEFAULT_DEBUG = false;

  ModelGenerator();

  /*!
   * \brief Cleans up a ModelGenerator.
   *
   * Cleans up any connections used by the ModelGenerator.
   */
  virtual ~ModelGenerator();

  /*!
   * \brief A check for a valid ModelGenerator.
   *
   * This function will return true if the appropriate connections were created successfully during initialization.
   *
   * \return True if the appropriate connections were created successfully during initialization.
   */
  bool okay() const;

  /**
  * Register all possible point cloud pairs
  */
  void pairwiseRegisterPointclouds();

  /**
  * Registered point clouds using a graph-based approach to successive pairwise registration
  * @param models models to register with the graph method
  * @return the number of generated models
  */
  int registerPointCloudsGraph(std::vector<Model> models, int maxModelSize, std::vector<int> unusedModelIds);

private:
  /*! The debug flag. */
  bool debug_, okay_;
  /*! The grasp database connection. */
  graspdb::Client *graspdb_;
  /*! Various ranking and matching metrics for point clouds. */
  PointCloudMetrics metrics_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The main model generation action server. */
  actionlib::SimpleActionServer<rail_pick_and_place_msgs::GenerateModelsAction> as_;


  void generateModelsCallback(const rail_pick_and_place_msgs::GenerateModelsGoalConstPtr &goal);


  //Point clouds
  sensor_msgs::PointCloud2 baseCloud;
  sensor_msgs::PointCloud2 targetCloud;

  //tf
  tf::TransformListener tfListener;
  tf::TransformBroadcaster tfBroadcaster;

  std::string outputDirectory;

  std::vector<Model> individualGraspModels;
  std::vector<Model> mergedModels;


  /**
   * Run pcl's icp on a base and target point cloud
   * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
   * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
   * @param baseGrasps list of grasps associated with the base point cloud
   * @param targetGrasps list of grasps associated with the target point cloud
   * @param resultGrasps pointer to the list of grasps that will be filled with transformed grasps
   * @return a pointer to the merged point cloud
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr,
      std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> baseGrasps,
      std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> targetGrasps,
      std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> *resultGrasps);

  /**
   * Determine if a point cloud registration is successful based on various score metrics
   * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
   * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
   */
  bool checkRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);

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
   * @param numNeighborThreshold minimum number of neighbors required to keep the point
   */
  void filterCloudOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, double radius, int numNeighborThreshold);

  /**
   * Removes extra points that are within a given threshold of other points to keep the point cloud size manageable
   * @param cloudPtr pointer to the point cloud to be filtered
   * @param dstThreshold the minimum distance between neighboring points at which to keep a point
   */
  void filterRedundentPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, double dstThreshold);

  /**
   * Translates a point cloud so that it's average point lies on the origin
   * @param cloudPtr pointer to the point cloud to be translated
   * @param grasps pointer to the grasps corresponding to the point cloud to be translated
   */
  void translateToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr,
      std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> *grasps);

  /**
   * Classify a point cloud merge as successful or not, based on a decision tree learned previously
   * @param overlap metric representing the overlap of the two point clouds
   * @param maxDstDiff metric representing the difference in maximum spatial lengths of the two point clouds
   * @param dstError metric representing the error in position of points in the two point clouds
   * @param avgColorDiff metric representing the difference in the colors of the point clouds
   */
  bool classifyMerge(float overlap, float maxDstDiff, float dstError, float avgColorDiff);
};

}
}
