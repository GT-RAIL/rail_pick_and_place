/*!
 * \file ObjectRecognitionListener.h
 * \brief The object recognition listener node object.
 *
 * The object recognition listener will listen to a specified SegmentedObjectsArray topic and attempt to recognize
 * all segmented objects. The new list are republished on a separate topic.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_OBJECT_RECOGNITION_LISTENER_H_
#define RAIL_PICK_AND_PLACE_OBJECT_RECOGNITION_LISTENER_H_

// ROS
#include <graspdb/graspdb.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>

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

  /*! The debug and okay check flags. */
  bool debug_, okay_;
  /*! The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The listener for the segmented objects. */
  ros::Subscriber segmented_objects_sub_;
  /*! The recognized objects and debug publishers. */
  ros::Publisher recognized_objects_pub_, debug_pub_;
  /*! The most recent segmented objects. */
  rail_manipulation_msgs::SegmentedObjectList object_list_;
};

}
}

#endif
