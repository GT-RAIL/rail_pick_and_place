/*!
 * \file PointCloudRecognizer.h
 * \brief The main recognition object for segmented point clouds.
 *
 * The point cloud recognizer takes a segmented object and a list of grasp model candidates and attempts to recognize
 * the object.
 *
 * \author David Kent, WPI - rctoris@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_POINT_CLOUD_RECOGNIZER_H_
#define RAIL_PICK_AND_PLACE_POINT_CLOUD_RECOGNIZER_H_

// RAIL Recognition
#include "PCLGraspModel.h"

// ROS
#include <graspdb/graspdb.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/ros.h>
#include <rail_manipulation_msgs/SegmentedObject.h>
#include <tf2/LinearMath/Transform.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rail
{
namespace pick_and_place
{

/*!
 * \class PointCloudRecognizer
 * \brief The main recognition object for segmented point clouds.
 *
 * The point cloud recognizer takes a segmented object and a list of grasp model candidates and attempts to recognize
 * the object.
 */
class PointCloudRecognizer
{
public:
  /*! The weighting constant for the match score metric. */
  static const double ALPHA = 0.5;
  /*! The confidence threshold for a recognition score. */
  static const double SCORE_CONFIDENCE_THRESHOLD = 0.8;
  /*! The threshold for the overlap metric to be considered a valid match. */
  static const double OVERLAP_THRESHOLD = 0.75;

  /*!
   * \brief Creates a new PointCloudRecognizer.
   *
   * Creates a new PointCloudRecognizer.
   */
  PointCloudRecognizer();

  /*!
   * \brief The main recognition function.
   *
   * Attempt to recognize the given object against the list of candidates. The object is compared to each candidate
   * and registration metrics are calculated. The weighted score is checked and the model with the lowest error score
   * is picked as the object. If this score meets the threshold, the segmented object is updated with the correct
   * grasps and object information.
   *
   * \param object The segmented object to recognize and update if recognition is successful.
   * \param candidates The list of candidate models for this object.
   * \return True if the segmented object was recognized and updated accordingly.
   */
  bool recognizeObject(rail_manipulation_msgs::SegmentedObject &object,
      const std::vector<PCLGraspModel> &candidates) const;

private:
  /*!
   * \brief Score the point cloud registration for the two point clouds.
   *
   * Perform registration from the object to the candidate and return the resulting weighted registration score (a
   * measure of error). The tf_icp transform is filled with the transform used to shift the object to the candidate.
   * A score of infinity (meaning a very poor match) is possible.
   *
   * \param candidate The candidate point cloud.
   * \param object The point cloud of the object in question.
   * \param tf_icp The transform from object to candidate used after ICP.
   * \return The score representing the weighted success of the registration (a measure of error).
   */
  double scoreRegistration(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr candidate,
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr object, tf2::Transform &tf_icp) const;

  /*!
   * \brief Compute the grasps for the recognized object.
   *
   * Compute the grasps to the recognized object based on the grasps from the model.
   *
   * \param tf_icp The transform between the candidate model and the segmented object.
   * \param centroid The centroid of the object point cloud.
   * \param candidate_grasps The candidate grasps from the model.
   * \param grasps The transformed grasps with respect to the recognized object.
   */
  void computeGraspList(const tf2::Transform &tf_icp, const geometry_msgs::Point &centroid,
      const std::vector<graspdb::Grasp> &candidate_grasps, std::vector<graspdb::Grasp> &grasps) const;
};

}
}

#endif
