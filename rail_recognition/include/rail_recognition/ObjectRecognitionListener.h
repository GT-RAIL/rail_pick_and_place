#ifndef OBJECT_RECOGNITION_LISTENER_H_
#define OBJECT_RECOGNITION_LISTENER_H_

//ROS
#include <ros/ros.h>
#include <graspdb/graspdb.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>

namespace rail
{
namespace pick_and_place
{

class ObjectRecognitionListener
{
public:

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
  void segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg);

  bool comparePointClouds(const sensor_msgs::PointCloud2 &pc1, const sensor_msgs::PointCloud2 &pc2) const;

  /*! The okay check flag. */
  bool okay_;
  /* The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The listener for the segmented objects. */
  ros::Subscriber segmented_objects_sub_;
  /*! The recognized objects publisher. */
  ros::Publisher recognized_objects_pub_;
  /*! The most recent segmented objects. */
  rail_manipulation_msgs::SegmentedObjectList object_list_;
};

}
}

#endif
