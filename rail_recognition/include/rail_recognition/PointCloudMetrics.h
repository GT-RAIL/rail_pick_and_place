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

void filterPointCloudOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    const double filter_outlier_search_radius = DEFAULT_FILTER_OUTLIER_SEARCH_RADIUS,
    const double filter_outlier_min_num_neighbors = DEFAULT_FILTER_OUTLIER_MIN_NUM_NEIGHBORS);

void filterRedundantPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    const double filter_redundant_search_radius = DEFAULT_FILTER_REDUNDANT_SEARCH_RADIUS);

geometry_msgs::Point computeCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc);

void transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc, const geometry_msgs::Point &centroid);

void transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc);

void transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    std::vector<graspdb::Grasp> &grasps, const geometry_msgs::Point &centroid);

void transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    std::vector<graspdb::Grasp> &grasps);

double calculateRegistrationMetricDistanceError(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target);

double calculateRegistrationMetricOverlap(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target, const bool return_color_error = false,
    const double metric_overlap_search_radius = DEFAULT_METRIC_OVERLAP_SEARCH_RADIUS);

double calculateRegistrationMetricColorRange(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target);

double calculateAverageColor(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc);

double calculateRegistrationMetricStdDevColorRange(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target);

double calculateStdDevColor(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc);

double calculateRegistrationMetricDistance(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target);

double calculateMaxDistance(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc);

bool classifyMerge(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target);

tf2::Transform performICP(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &source,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &result);

}
}
}

#endif
