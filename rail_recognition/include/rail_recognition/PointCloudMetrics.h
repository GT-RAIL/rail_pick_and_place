/*!
 * \file PointCloudMetrics.h
 * \brief Various point cloud metric calculations and utilities.
 *
 * A collection of static functions for calculating various point cloud metrics and utility functions.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_POINT_CLOUD_METRICS_H_
#define RAIL_PICK_AND_PLACE_POINT_CLOUD_METRICS_H_

// ROS
#include <geometry_msgs/Point.h>
#include <graspdb/Grasp.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// C++ Standard Library
#include <vector>

namespace rail
{
namespace pick_and_place
{
namespace point_cloud_metrics
{

/*! The radius to search within for neighbors during pre-processing. */
static const double DEFAULT_FILTER_OUTLIER_SEARCH_RADIUS = 0.01;
/*! The radius to search within for neighbors during pre-processing. */
static const double DEFAULT_FILTER_REDUNDANT_SEARCH_RADIUS = 0.00075;
/*! The minimum number of neighbors required to keep a point during pre-processing. */
static const int DEFAULT_FILTER_OUTLIER_MIN_NUM_NEIGHBORS = 6;
/*! The radius to search within for neighbors during the overlap metric search. */
static const double DEFAULT_METRIC_OVERLAP_SEARCH_RADIUS = 0.005;

/*!
 * \brief Convert a ROS point cloud message to a PCL point cloud.
 *
 * Converts the given ROS point cloud message to a PCL point cloud.
 *
 * \param in The input ROS point cloud message.
 * \param out The PCL point cloud to create.
 */
void rosPointCloud2ToPCLPointCloud(const sensor_msgs::PointCloud2 in,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out);

/*!
 * \brief Convert a PCL point cloud to a ROS point cloud message.
 *
 * Converts the given PCL point cloud to a ROS point cloud message.
 *
 * \param in The input PCL point cloud message.
 * \param out The ROS point cloud message to create.
 */
void pclPointCloudToROSPointCloud2(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in,
    sensor_msgs::PointCloud2 &out);

/*!
 * \brief Filter point cloud outliers.
 *
 * Filter outliers in the point cloud.
 *
 * \param pc The point cloud to remove outlier points from.
 * \param filter_outlier_search_radius The search radius to be considered as an outlier point (defaults to constant).
 * \param filter_outlier_min_num_neighbors The minimum neighbors to be consider as an outlier (defaults to constant).
 * \return The centroid of the point cloud.
 */
void filterPointCloudOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    const double filter_outlier_search_radius = DEFAULT_FILTER_OUTLIER_SEARCH_RADIUS,
    const double filter_outlier_min_num_neighbors = DEFAULT_FILTER_OUTLIER_MIN_NUM_NEIGHBORS);

/*!
 * \brief Filter redundant points from the point cloud.
 *
 * Filter redundant points from the point cloud effectively downsampling it.
 *
 * \param pc The point cloud to remove redundant points from.
 * \param filter_redundant_search_radius The search radius to be considered as a redundant point (defaults to constant).
 */
void filterRedundantPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    const double filter_redundant_search_radius = DEFAULT_FILTER_REDUNDANT_SEARCH_RADIUS);

/*!
 * \brief Compute the centroid of the given point cloud.
 *
 * Compute and return the centroid of the given point cloud.
 *
 * \param pc The point cloud to compute the centroid of.
 * \return The centroid of the point cloud.
 */
geometry_msgs::Point computeCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc);

/*!
 * \brief Transform the point cloud to be centered about the origin.
 *
 * Transform the point cloud to be centered about the origin based on the given centroid of the point cloud.
 *
 * \param pc The point cloud to transform.
 * \param centroid The known centroid of the point cloud.
 */
void transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc, const geometry_msgs::Point &centroid);

/*!
 * \brief Transform the point cloud to be centered about the origin.
 *
 * Transform the point cloud to be centered about the origin based on the centroid of the point cloud.
 *
 * \param pc The point cloud to transform.
 */
void transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc);

/*!
 * \brief Transform the point cloud and grasps to be centered about the origin.
 *
 * Transform the point cloud and grasps to be centered about the origin based on the given centroid of the point cloud.
 *
 * \param pc The point cloud to transform.
 * \param grasps The grasps to transform.
 * \param centroid The known centroid of the point cloud.
 */
void transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    std::vector<graspdb::Grasp> &grasps, const geometry_msgs::Point &centroid);

/*!
 * \brief Transform the point cloud and grasps to be centered about the origin.
 *
 * Transform the point cloud and grasps to be centered about the origin based on the centroid of the point cloud.
 *
 * \param pc The point cloud to transform.
 * \param grasps The grasps to transform.
 */
void transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    std::vector<graspdb::Grasp> &grasps);

/*!
 * \brief Point cloud distance error metric calculator.
 *
 * Calculate the total distance between each point in target to the closest point in base.
 *
 * \param base The base point cloud.
 * \param target The target point cloud..
 * \return The total distance between each point in target to the closest point in base.
 */
double calculateRegistrationMetricDistanceError(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target);

/*!
 * \brief Point cloud overlap metric calculator.
 *
 * Calculate the overlap metric for the given point clouds. This can either be the number of points that meet the
 * overlap criteria normalized over the number of points, or the average error in the RGB color for the overlap
 * points. References to each double to be filled must be provided.
 *
 * \param base The base point cloud.
 * \param target The target point cloud.
 * \param overlap The overlap metric score.
 * \param color_error The color error metric score.
 * \param metric_overlap_search_radius The search radius to consider a point to be overlapping (defaults to constant).
 */
void calculateRegistrationMetricOverlap(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target, double &overlap, double &color_error,
    const double metric_overlap_search_radius = DEFAULT_METRIC_OVERLAP_SEARCH_RADIUS);

/*!
 * \brief Average color value calculator.
 *
 * Calculate the average red, green, and blue values of the point cloud.
 *
 * \param pc The point cloud.
 * \param avg_r A reference to the average red value to store.
 * \param avg_g A reference to the average green value to store.
 * \param avg_b A reference to the average blue value to store.
 */
void calculateAvgColors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc, double &avg_r, double &avg_g,
    double &avg_b);

/*!
 * \brief Color value standard deviation calculator.
 *
 * Calculate the red, green, and blue value standard deviations of the point cloud.
 *
 * \param pc The point cloud.
 * \param std_dev_r A reference to the standard deviation in the red value to store.
 * \param std_dev_g A reference to the standard deviation in the green value to store.
 * \param std_dev_b A reference to the standard deviation in the blue value to store.
 */
double calculateStdDevColors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc, double &std_dev_r,
    double &std_dev_g, double &std_dev_b);

/*!
 * \brief Color value standard deviation calculator.
 *
 * Calculate the red, green, and blue value standard deviations of the point cloud with the known averages.
 *
 * \param pc The point cloud.
 * \param std_dev_r A reference to the standard deviation in the red value to store.
 * \param std_dev_g A reference to the standard deviation in the green value to store.
 * \param std_dev_b A reference to the standard deviation in the blue value to store.
 * \param avg_r The average red color.
 * \param avg_g The average green color.
 * \param avg_b The average blue color.
 */
double calculateStdDevColors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc, double &std_dev_r,
    double &std_dev_g, double &std_dev_b, const double avg_r, const double avg_g, const double avg_b);

/*!
 * \brief Classify the point cloud merge.
 *
 * Classify the merge between base and target. These point clouds should already be transformed accordingly. The
 * decision is based on values from a trained decision tree on point cloud metrics.
 *
 * \param base The base point cloud.
 * \param target The target point cloud.
 * \return If the calculated metrics meet the criteria for a valid merge.
 */
bool classifyMerge(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target);

/*!
 * \brief Perform ICP on the given point clouds.
 *
 * Perform ICP on the given point clouds and store the resulting transformed point cloud in the result pointer.
 *
 * \param target The target point cloud.
 * \param source The source point cloud (to transform to the target).
 * \param result The transformed source point cloud.
 * \return The transform used to move source to target.
 */
tf2::Transform performICP(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &result);
}
}
}

#endif
