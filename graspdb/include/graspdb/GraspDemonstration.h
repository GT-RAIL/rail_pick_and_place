/*!
 * \file GraspDemonstration.h
 * \brief A grasp demonstration database entity.
 *
 * A grasp demonstration contains information about a single grasp demonstration in the grasp database. This
 * contains information about the grasp pose, end effector frame identifier, object name, and serialized segmented
 * point cloud. A valid database entity has an ID and created timestamp. This class is useful for internal data
 * management within the graspdb library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASPDB_GRASP_DEMONSTRATION_H_
#define RAIL_PICK_AND_PLACE_GRASPDB_GRASP_DEMONSTRATION_H_

// graspdb
#include "Entity.h"
#include "Pose.h"

// ROS
#include <rail_pick_and_place_msgs/GraspDemonstration.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// C++ Standard Library
#include <string>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

/*!
 * \class GraspDemonstration
 * \brief A grasp demonstration database entry.
 *
 * A grasp demonstration contains information about a single grasp demonstration in the grasp database. This
 * contains information about the grasp pose, end effector frame identifier, object name, and serialized segmented
 * point cloud. A valid database entity has an ID and created timestamp. This class is useful for internal data
 * management within the graspdb library. Convenience functions are added for use with ROS messages.
 */
class GraspDemonstration : public Entity
{
public:
  /*!
   * \brief Create a new GraspDemonstration.
   *
   * Creates a new GraspDemonstration with the given values. This constructor assumes a valid ID and timestamp are
   * known.
   *
   * \param id The unique ID of the database entry (defaults to 0).
   * \param object_name The name of the object grasped (defaults to the empty string).
   * \param grasp_pose The pose of the grasp (defaults to an empty Pose).
   * \param eef_frame_id The frame identifier for the end effector used (defaults to the empty string).
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object (defaults to empty values).
   * \param image The ROS sensor_msgs/Image message of the segmented object (defaults to empty values).
   * \param created The created timestamp (defaults to 0).
   */
  GraspDemonstration(const uint32_t id = Entity::UNSET_ID, const std::string &object_name = "",
                     const Pose &grasp_pose = Pose(), const std::string &eef_frame_id = "",
                     const sensor_msgs::PointCloud2 &point_cloud = sensor_msgs::PointCloud2(),
                     const sensor_msgs::Image &image = sensor_msgs::Image(), const time_t created = Entity::UNSET_TIME);

  /*!
   * \brief Create a new GraspDemonstration.
   *
   * Creates a new GraspDemonstration with the given values. This constructor assumes no valid ID and timestamp are
   * known (e.g., for use when inserting into the database).
   *
   * \param object_name The name of the object grasped.
   * \param grasp_pose The pose of the grasp.
   * \param eef_frame_id The frame identifier for the end effector used.
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object.
   * \param image The ROS sensor_msgs/Image message of the segmented object.
   */
  GraspDemonstration(const std::string &object_name, const Pose &grasp_pose, const std::string &eef_frame_id,
                     const sensor_msgs::PointCloud2 &point_cloud, const sensor_msgs::Image &image);

  /*!
   * \brief Create a new GraspDemonstration.
   *
   * Creates a new GraspDemonstration with the given values the ROS message.
   *
   * \param gd The ROS grasp demonstration message to extract values from.
   */
  GraspDemonstration(const rail_pick_and_place_msgs::GraspDemonstration &gd);

  /*!
   * \brief Object name value accessor.
   *
   * Get the object name value of this GraspDemonstration.
   *
   * \return The object name value.
   */
  const std::string &getObjectName() const;

  /*!
   * \brief Object name value mutator.
   *
   * Set the object name value of this GraspDemonstration.
   *
   * \param object_name The new object name value.
   */
  void setObjectName(const std::string &object_name);

  /*!
   * \brief Grasp pose value accessor (immutable).
   *
   * Get the grasp pose value of this GraspDemonstration.
   *
   * \return The grasp pose value.
   */
  const Pose &getGraspPose() const;

  /*!
   * \brief Grasp pose value accessor.
   *
   * Get the grasp pose value of this GraspDemonstration.
   *
   * \return The grasp pose value.
   */
  Pose &getGraspPose();

  /*!
   * \brief Grasp pose value mutator.
   *
   * Set the grasp pose value of this GraspDemonstration.
   *
   * \param grasp_pose The new grasp pose value.
   */
  void setGraspPose(const Pose &grasp_pose);

  /*!
   * \brief End effector frame ID value accessor.
   *
   * Get the end effector frame ID value of this GraspDemonstration.
   *
   * \return The end effector frame ID value.
   */
  const std::string &getEefFrameID() const;

  /*!
   * \brief End effector frame ID value mutator.
   *
   * Set the end effector frame ID value of this GraspDemonstration.
   *
   * \param eef_frame_id The new object name value.
   */
  void setEefFrameID(const std::string &eef_frame_id);

  /*!
   * \brief Point cloud accessor (immutable).
   *
   * Get the point cloud message.
   *
   * \return The point cloud message.
   */
  const sensor_msgs::PointCloud2 &getPointCloud() const;

  /*!
   * \brief Point cloud accessor.
   *
   * Get the point cloud message.
   *
   * \return The point cloud message.
   */
  sensor_msgs::PointCloud2 &getPointCloud();

  /*!
   * \brief Point cloud mutator.
   *
   * Set the point cloud message to the given values based on the ROS message.
   *
   * \param point_cloud The ROS PointCloud2 message to store.
   */
  void setPointCloud(const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Image accessor (immutable).
   *
   * Get the image message.
   *
   * \return The image message.
   */
  const sensor_msgs::Image &getImage() const;

  /*!
   * \brief Image accessor.
   *
   * Get the image message.
   *
   * \return The image message.
   */
  sensor_msgs::Image &getImage();

  /*!
   * \brief Image mutator.
   *
   * Set the image message to the given values based on the ROS message.
   *
   * \param image The ROS Image message to store.
   */
  void setImage(const sensor_msgs::Image &image);

  /*!
   * Converts this GraspDemonstration object into a ROS GraspDemonstration message.
   *
   * \return The ROS GraspDemonstration message with this grasp demonstration data.
   */
  rail_pick_and_place_msgs::GraspDemonstration toROSGraspDemonstrationMessage() const;

private:
  /*! The name of the object and end effector frame identifier for this demonstration entry. */
  std::string object_name_, eef_frame_id_;
  /*! The grasp pose data. */
  Pose grasp_pose_;
  /*! The point cloud data. */
  sensor_msgs::PointCloud2 point_cloud_;
  /*! The RGB image data. */
  sensor_msgs::Image image_;
};

}
}
}

#endif
