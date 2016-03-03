/*!
 * \file ObjectRecognitionListener.h
 * \brief The object recognition listener node object.
 *
 * The object recognition listener will listen to a specified SegmentedObjectsArray topic and attempt to recognize
 * all segmented objects. The new list are republished on a separate topic.
 *
 * \author Russell Toris, WPI - russell.toris@gmail.com
 * \author David Kent, WPI - russell.toris@gmail.com
 * \date April 8, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_OBJECT_RECOGNITION_LISTENER_H_
#define RAIL_PICK_AND_PLACE_OBJECT_RECOGNITION_LISTENER_H_

// ROS
#include <graspdb/graspdb.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_pick_and_place_msgs/RemoveObject.h>
#include <rail_recognition/ImageRecognizer.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>

// Boost
#include <boost/thread/mutex.hpp>

namespace rail
{
namespace pick_and_place
{

/*!
 * \class ObjectRecognitionListener
 * \brief The object recognition listener node object.
 *
 * The object recognition listener will listen to a specified SegmentedObjectsArray topic and attempt to recognize
 * all segmented objects. The new list are republished on a separate topic.
 */
class ObjectRecognitionListener
{
public:
  /*! If a topic should be created to display debug information such as pose arrays. */
  static const bool DEFAULT_DEBUG = false;
  /*! Leaf size of the voxel grid for downsampling. */
  static const float DOWNSAMPLE_LEAF_SIZE = 0.01;
  /*! The distance threshold for two point clouds being the same item. */
  static const double SAME_OBJECT_DIST_THRESHOLD = 0.2;
  /*! Confidence threshold for the recognizer. */
  static const double RECOGNITION_THRESHOLD = 0.5;

  /*!
   * \brief Creates a new ObjectRecognitionListener.
   *
   * Creates a new ObjectRecognitionListener with the associated topics.
   */
  ObjectRecognitionListener();

  /*!
   * \brief Cleans up a ObjectRecognitionListener.
   *
   * Cleans up any connections used by the ObjectRecognitionListener.
   */
  virtual ~ObjectRecognitionListener();

  /*!
   * \brief A check for a valid ObjectRecognitionListener.
   *
   * This function will return true if the appropriate connections were created successfully during initialization.
   *
   * \return True if the appropriate connections were created successfully during initialization.
   */
  bool okay() const;

private:
  /*!
   * \brief The segmented objects callback.
   *
   * Take the current list of segmented objects and attempt to recognize each one. The new list is republished on the
   * recognized objects topic.
   *
   * \param objects The current list of segmented objects.
   */
  void segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &objects);

  /*!
   * \brief Check if the two point clouds are the same.
   *
   * Checks if the data in the two point clouds are the same.
   *
   * \param pc1 The first point cloud.
   * \param pc2 The second point cloud.
   * \return Returns true if the data in pc1 is equal to the data in pc2.
   */
  bool comparePointClouds(const sensor_msgs::PointCloud2 &pc1, const sensor_msgs::PointCloud2 &pc2) const;

  /*!
   * \brief Combine two models of the same type into a single model.
   *
   * Take the two segmented object models and combine them into a single model. This is useful for when a single
   * object has mistakenly been split into two models.
   *
   * \param model1 The first object model.
   * \param model2 The second object model.
   * \param out The combined model to set.
   */
  void combineModels(const rail_manipulation_msgs::SegmentedObject &model1,
      const rail_manipulation_msgs::SegmentedObject &model2, rail_manipulation_msgs::SegmentedObject &combined) const;

  /*!
   * \brief Callback for the remove object request.
   *
   * Remote the object from the segmented object list with a given ID. This will publish both an updated segmented
   * object list.
   *
   * \param req The request with the index to use.
   * \param res The empty response (unused).
   * \return Returns true if a valid index was provided.
   */
  bool removeObjectCallback(rail_pick_and_place_msgs::RemoveObject::Request &req,
      rail_pick_and_place_msgs::RemoveObject::Response &res);

  /*! The debug and okay check flags. */
  bool debug_, okay_;
  /*! The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The main mutex for list modifications. */
  boost::mutex mutex_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The listener for the segmented objects. */
  ros::Subscriber segmented_objects_sub_;
  /*! The recognized objects and debug publishers. */
  ros::Publisher recognized_objects_pub_, debug_pub_;
  /*! The remove object server. */
  ros::ServiceServer remove_object_srv_;
  /*! The most recent segmented objects. */
  rail_manipulation_msgs::SegmentedObjectList object_list_;

  ImageRecognizer image_recognizer_;

  bool use_image_recognition_;
};

}
}

#endif
