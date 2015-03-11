/*!
 * \file GraspDemonstration.h
 * \brief A grasp demonstration database entry.
 *
 * A grasp demonstration contains information about a single grasp demonstration in the grasp database. This
 * contains information about the grasp pose, object name, and serialized segmented point cloud.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

#ifndef RAIL_GRASPDB_GRASP_DEMONSTRATION_H_
#define RAIL_GRASPDB_GRASP_DEMONSTRATION_H_

#include <geometry_msgs/Pose.h>
#include <graspdb/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <rail_pick_and_place_msgs/GraspDemonstration.h>
#include <stdint.h>
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
 * contains information about the grasp pose, object name, and serialized segmented point cloud.
 */
class GraspDemonstration
{
public:
  /*! The default value for an unset identifier (i.e., a demonstration not yet in the database). */
  static const uint32_t UNSET_ID = 0;
  /*! The default value for an unset timestamp (i.e., a demonstration not yet in the database). */
  static const time_t UNSET_TIME = 0;

  /*!
   * \brief Create a new GraspDemonstration.
   *
   * Creates a new GraspDemonstration with the given values. This constructor assumes a valid ID and timestamp are
   * known.
   *
   * \param id The unique ID of the database entry (defaults to 0).
   * \param object_name The name of the object grasped (defaults to the empty string).
   * \param grasp_pose The pose of the grasp (defaults to an empty Pose).
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object (defaults to empty values).
   * \param created The created timestamp (defaults to 0).
   */
  GraspDemonstration(const uint32_t id = UNSET_ID, const std::string &object_name = "", const Pose &grasp_pose = Pose(),
      const sensor_msgs::PointCloud2 &point_cloud = sensor_msgs::PointCloud2(), const time_t created = UNSET_TIME);

  /*!
   * \brief Create a new GraspDemonstration.
   *
   * Creates a new GraspDemonstration with the given values. This constructor assumes no valid ID and timestamp are
   * known (e.g., for use when inserting into the database).
   *
   * \param object_name The name of the object grasped.
   * \param grasp_pose The pose of the grasp.
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object (defaults to empty values).
   */
  GraspDemonstration(const std::string &object_name, const Pose &grasp_pose,
      const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Create a new GraspDemonstration.
   *
   * Creates a new GraspDemonstration with the given values from ROS messages. This constructor assumes no valid ID
   * and timestamp are known (e.g., for use when inserting into the database).
   *
   * \param object_name The name of the object grasped.
   * \param grasp_pose_fixed_frame_id The name of the frame the grasp pose is in reference to.
   * \param grasp_pose_grasp_frame_id The name of the frame for the grasp pose.
   * \param grasp_pose The ROS Pose message to extract pose data of the grasp from.
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object to be serialized into a buffer.
   */
  GraspDemonstration(const std::string &object_name, const std::string &grasp_pose_fixed_frame_id,
      const std::string &grasp_pose_grasp_frame_id, const geometry_msgs::Pose &grasp_pose,
      const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Create a new GraspDemonstration.
   *
   * Creates a new GraspDemonstration with the given values from ROS messages. This constructor assumes no valid ID
   * and timestamp are known (e.g., for use when inserting into the database).
   *
   * \param object_name The name of the object grasped.
   * \param grasp_pose_fixed_frame_id The name of the frame the grasp pose is in reference to.
   * \param grasp_pose_grasp_frame_id The name of the frame for the grasp pose.
   * \param grasp_pose The ROS Transform message to extract pose data of the grasp from.
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object to be serialized into a buffer.
   */
  GraspDemonstration(const std::string &object_name, const std::string &grasp_pose_fixed_frame_id,
      const std::string &grasp_pose_grasp_frame_id, const geometry_msgs::Transform &grasp_pose,
      const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Create a new GraspDemonstration.
   *
   * Creates a new GraspDemonstration with the given values the ROS message.
   *
   * \param gd The ROS grasp demonstration message to extract values from.
   */
  GraspDemonstration(const rail_pick_and_place_msgs::GraspDemonstration &gd);

  /*!
   * \brief ID value accessor.
   *
   * Get the ID value of this GraspDemonstration.
   *
   * \return The ID value.
   */
  uint32_t getID() const;

  /*!
   * \brief ID value mutator.
   *
   * Set the ID value of this GraspDemonstration.
   *
   * \param id The new ID value.
   */
  void setID(const uint32_t id);

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
   * \brief Grasp pose value accessor.
   *
   * Get the grasp pose value of this GraspDemonstration.
   *
   * \return The grasp pose value.
   */
  const Pose &getGraspPose() const;

  /*!
   * \brief Grasp pose value mutator.
   *
   * Set the grasp pose value of this GraspDemonstration.
   *
   * \param grasp_pose The new grasp pose value.
   */
  void setGraspPose(const Pose &grasp_pose);

  /*!
   * \brief Point cloud accessor.
   *
   * Get the point cloud message.
   *
   * \return The point cloud message.
   */
  const sensor_msgs::PointCloud2 &getPointCloud() const;

  /*!
   * \brief Point cloud buffer mutator.
   *
   * Set the point cloud message to the given values based on the ROS message.
   *
   * \param point_cloud The ROS PointCloud2 messsage to store.
   */
  void setPointCloud(const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Created timestamp value accessor.
   *
   * Get the created timestamp value of this GraspDemonstration.
   *
   * \return The created timestamp value.
   */
  time_t getCreated() const;

  /*!
   * \brief Created timestamp value mutator.
   *
   * Set the created timestamp value of this GraspDemonstration.
   *
   * \param created The new created timestamp value.
   */
  void setCreated(const time_t created);

  /*!
   * Converts this GraspDemonstration object into a ROS GraspDemonstration message.
   *
   * \return The ROS GraspDemonstration message with this grasp demonstration data.
   */
  rail_pick_and_place_msgs::GraspDemonstration toROSPGraspDemonstrationMessage() const;

private:
  /*! The ID. */
  uint32_t id_;
  /*! The name of the object for this demonstration entry. */
  std::string object_name_;
  /*! The grasp pose data. */
  Pose grasp_pose_;
  /*! The serialized point cloud buffer. */
  sensor_msgs::PointCloud2 point_cloud_;
  /*! The created timestamp. */
  time_t created_;
};

}
}
}

#endif
