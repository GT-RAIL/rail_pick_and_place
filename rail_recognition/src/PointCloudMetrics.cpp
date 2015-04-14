/*!
 * \file PointCloudMetrics.cpp
 * \brief Various point cloud metric calculations and utilities.
 *
 * A collection of static functions for calculating various point cloud metrics and utility functions.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

// RAIL Recognition
#include "rail_recognition/PointCloudMetrics.h"

// ROS
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

using namespace std;
using namespace rail::pick_and_place;

void point_cloud_metrics::rosPointCloud2ToPCLPointCloud(const sensor_msgs::PointCloud2 in,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out)
{
  pcl::PCLPointCloud2 converter;
  pcl_conversions::toPCL(in, converter);
  pcl::fromPCLPointCloud2(converter, *out);
}

void point_cloud_metrics::pclPointCloudToROSPointCloud2(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in,
    sensor_msgs::PointCloud2 &out)
{
  // convert to PCL PointCloud2
  pcl::PCLPointCloud2 converter;
  pcl::toPCLPointCloud2(*in, converter);
  // convert to ROS message
  pcl_conversions::fromPCL(converter, out);
}

void point_cloud_metrics::transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    const geometry_msgs::Point &centroid)
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

void point_cloud_metrics::transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc)
{
  // compute the centroid
  geometry_msgs::Point centroid = point_cloud_metrics::computeCentroid(pc);
  // transform the point cloud
  point_cloud_metrics::transformToOrigin(pc, centroid);
}

void point_cloud_metrics::transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    vector<graspdb::Grasp> &grasps, const geometry_msgs::Point &centroid)
{
  // start with the point cloud
  point_cloud_metrics::transformToOrigin(pc, centroid);

  // transform each grasp
  for (size_t i = 0; i < grasps.size(); i++)
  {
    graspdb::Position &position = grasps[i].getGraspPose().getPosition();
    position.setX(position.getX() - centroid.x);
    position.setY(position.getY() - centroid.y);
    position.setZ(position.getZ() - centroid.z);
  }
}

void point_cloud_metrics::transformToOrigin(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    vector<graspdb::Grasp> &grasps)
{
  // compute the centroid
  geometry_msgs::Point centroid = point_cloud_metrics::computeCentroid(pc);
  // transform the point cloud and grasps
  point_cloud_metrics::transformToOrigin(pc, grasps, centroid);
}

void point_cloud_metrics::filterPointCloudOutliers(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    const double filter_outlier_search_radius, const double filter_outlier_min_num_neighbors)
{
  // use a KD tree to search
  pcl::KdTreeFLANN<pcl::PointXYZRGB> search_tree;
  search_tree.setInputCloud(pc);
  vector<int> indices;
  vector<float> distances;

  // check each point
  pcl::IndicesPtr to_keep(new vector<int>);
  for (size_t i = 0; i < pc->size(); i++)
  {
    // check how many neighbors pass the test
    int neighbors = search_tree.radiusSearch(pc->at(i), filter_outlier_search_radius, indices, distances);
    if (neighbors >= filter_outlier_min_num_neighbors)
    {
      to_keep->push_back(i);
    }
  }

  // extract the point we wish to keep
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(pc);
  extract.setIndices(to_keep);
  extract.filter(*pc);
}

void point_cloud_metrics::filterRedundantPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc,
    const double filter_redundant_search_radius)
{
  // non-empty clouds only
  if (pc->size() > 0)
  {
    // use a KD tree to search
    pcl::KdTreeFLANN<pcl::PointXYZRGB> search_tree;
    search_tree.setInputCloud(pc);

    // use int to prevent overflow
    for (int i = ((int) pc->size()) - 1; i >= 0; i--)
    {
      // see if the number of neighbors is non-empty (includes the point itself)
      vector<int> indices;
      vector<float> distances;
      int neighbors = search_tree.radiusSearch(pc->at(i), filter_redundant_search_radius, indices, distances);
      if (neighbors >= 2)
      {
        // remove this point
        pc->erase(pc->begin() + i);
      }
    }
  }
}

geometry_msgs::Point point_cloud_metrics::computeCentroid(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc)
{
  // compute the centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*pc, centroid);
  geometry_msgs::Point ros_centroid;
  ros_centroid.x = centroid[0];
  ros_centroid.y = centroid[1];
  ros_centroid.z = centroid[2];

  return ros_centroid;
}

double point_cloud_metrics::calculateRegistrationMetricDistanceError(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target)
{
  // search using a KD tree
  pcl::KdTreeFLANN<pcl::PointXYZRGB> search_tree;
  search_tree.setInputCloud(base);

  // search for the nearest point to each point
  double score = 0;
  for (size_t i = 0; i < target->size(); i++)
  {
    vector<int> indices;
    vector<float> distances;
    search_tree.nearestKSearch(target->at(i), 1, indices, distances);
    score += (double) distances[0];
  }

  return score;
}

void point_cloud_metrics::calculateAvgColors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc, double &avg_r,
    double &avg_g, double &avg_b)
{
  // simply go through each point
  avg_r = 0;
  avg_g = 0;
  avg_b = 0;
  for (size_t i = 0; i < pc->size(); i++)
  {
    const pcl::PointXYZRGB &point = pc->at(i);
    avg_r += point.r;
    avg_g += point.g;
    avg_b += point.b;
  }

  // average the values
  avg_r /= (double) pc->size();
  avg_g /= (double) pc->size();
  avg_b /= (double) pc->size();
}

double point_cloud_metrics::calculateStdDevColors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc,
    double &std_dev_r, double &std_dev_g, double &std_dev_b)
{
  // calculate the averages first
  double avg_r, avg_g, avg_b;
  point_cloud_metrics::calculateAvgColors(pc, avg_r, avg_g, avg_b);
  point_cloud_metrics::calculateStdDevColors(pc, std_dev_r, std_dev_g, std_dev_b, avg_r, avg_g, avg_b);
}

double point_cloud_metrics::calculateStdDevColors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc,
    double &std_dev_r, double &std_dev_g, double &std_dev_b, const double avg_r, const double avg_g, const double avg_b)
{
  double variance_r = 0;
  double variance_g = 0;
  double variance_b = 0;
  for (size_t i = 0; i < pc->size(); i++)
  {
    const pcl::PointXYZRGB &point = pc->at(i);
    variance_r += pow(point.r - avg_r, 2);
    variance_g += pow(point.g - avg_r, 2);
    variance_b += pow(point.b - avg_r, 2);
  }
  variance_r /= (double) pc->size();
  variance_g /= (double) pc->size();
  variance_b /= (double) pc->size();

  std_dev_r = sqrt(variance_r);
  std_dev_g = sqrt(variance_g);
  std_dev_b = sqrt(variance_b);
}

void point_cloud_metrics::calculateRegistrationMetricOverlap(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target, double &overlap, double &color_error,
    const double metric_overlap_search_radius)
{
  // search with a KD tree
  pcl::KdTreeFLANN<pcl::PointXYZRGB> search_tree;
  search_tree.setInputCloud(base);

  // search each point
  double score = 0;
  double error = 0;
  for (size_t i = 0; i < target->size(); i++)
  {
    // get the current point
    vector<int> indices;
    vector<float> distances;
    const pcl::PointXYZRGB &search_point = target->at(i);
    // perform a radius search to see how many neighbors are found
    int neighbors = search_tree.radiusSearch(search_point, metric_overlap_search_radius, indices, distances);
    // check if there are enough neighbors
    if (neighbors > 0)
    {
      score++;
      // check the average RGB color distance
      double rgb_distance = 0;
      for (size_t j = 0; j < indices.size(); j++)
      {
        const pcl::PointXYZRGB &point = base->at(indices[j]);
        rgb_distance += sqrt(pow(search_point.r - point.r, 2) + pow(search_point.g - point.g, 2) +
                             pow(search_point.b - point.b, 2));
      }
      // normalize the distance
      rgb_distance /= neighbors;
      error += rgb_distance;
    }
  }

  // normalize the errors
  color_error = error / score;
  overlap = score / (double) target->size();
}

bool point_cloud_metrics::classifyMerge(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &base,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target)
{
  // calculate the metrics we need
  double overlap, color_error;
  point_cloud_metrics::calculateRegistrationMetricOverlap(base, target, overlap, color_error);

  // values found via decision tree training
  return (overlap > 0.471303) && (color_error <= 97.0674);
}

tf2::Transform point_cloud_metrics::performICP(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &target,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &result)
{
  // set the ICP point clouds
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  // run the alignment
  icp.align(*result);

  // get the resulting transform
  Eigen::Matrix4f transform = icp.getFinalTransformation();

  // tanslate to a TF2 transform
  tf2::Matrix3x3 rotation(transform(0, 0), transform(0, 1), transform(0, 2),
                          transform(1, 0), transform(1, 1), transform(1, 2),
                          transform(2, 0), transform(2, 1), transform(2, 2));
  tf2::Vector3 translation(transform(0, 3), transform(1, 3), transform(2, 3));
  tf2::Transform tf_icp(rotation, translation);
  return tf_icp;
}
