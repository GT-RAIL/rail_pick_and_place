/*!
 * \file GraspDemonstration.h
 * \brief A grasp demonstration database entry.
 *
 * A grasp demonstration contains information about a single grasp demonstration in the grasp database. This contains information about the grasp pose, object name, and serialized segmented point cloud.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

#ifndef RAIL_GRASPDB_GRASP_DEMONSTRATION_H_
#define RAIL_GRASPDB_GRASP_DEMONSTRATION_H_

#include <geometry_msgs/Pose.h>
#include <graspdb/Pose.h>
#include <sensor_msgs/PointCloud2.h>
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
 * A grasp demonstration contains information about a single grasp demonstration in the grasp database. This contains information about the grasp pose, object name, and serialized segmented point cloud.
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
   * Creates a new GraspDemonstration with the given values. This constructor assumes a valid ID and timestamp are known.
   *
   * \param id The unique ID of the database entry.
   * \param object_name The name of the object grasped.
   * \param grasp_pose The pose of the grasp.
   * \param point_cloud The serialized ROS sensor_msgs/PointCloud2 message of the segmented object.
   * \param point_cloud_size The size of the serialized point cloud buffer.
   * \param created The created timestamp.
   */
  GraspDemonstration(const uint32_t id, const std::string object_name, const Pose grasp_pose, const uint8_t *point_cloud, const uint32_t point_cloud_size, const time_t created);

  /*!
   * \brief Create a new GraspDemonstration.
   *
   * Creates a new GraspDemonstration with the given values. This constructor assumes no valid ID and timestamp are known (e.g., for use when inserting into the database).
   *
   * \param object_name The name of the object grasped.
   * \param grasp_pose The pose of the grasp.
   * \param point_cloud The serialized ROS sensor_msgs/PointCloud2 message of the segmented object.
   * \param point_cloud_size The size of the serialized point cloud buffer.
   */
  GraspDemonstration(const std::string object_name, const Pose grasp_pose, const uint8_t *point_cloud, const uint32_t point_cloud_size);

  /*!
   * \brief Create a new GraspDemonstration.
   *
   * Creates a new GraspDemonstration with the given values from ROS messages. This constructor assumes no valid ID and timestamp are known (e.g., for use when inserting into the database).
   *
   * \param object_name The name of the object grasped.
   * \param grasp_pose_frame_id The name of the frame the grasp pose is in reference to.
   * \param grasp_pose The ROS Pose message to extract pose data of the grasp from.
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object to be serialized into a buffer.
   */
  GraspDemonstration(const std::string object_name, const std::string grasp_pose_frame_id, const geometry_msgs::Pose &grasp_pose, const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Create a new GraspDemonstration.
   *
   * Creates a new GraspDemonstration with the given values from ROS messages. This constructor assumes no valid ID and timestamp are known (e.g., for use when inserting into the database).
   *
   * \param object_name The name of the object grasped.
   * \param grasp_pose_frame_id The name of the frame the grasp pose is in reference to.
   * \param grasp_pose The ROS Transform message to extract pose data of the grasp from.
   * \param point_cloud The ROS sensor_msgs/PointCloud2 message of the segmented object to be serialized into a buffer.
   */
  GraspDemonstration(const std::string object_name, const std::string grasp_pose_frame_id, const geometry_msgs::Transform &grasp_pose, const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Cleans up a GraspDemonstration.
   *
   * Cleans up any buffers used by the GraspDemonstration.
   */
  ~GraspDemonstration();

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
  void setObjectName(const std::string object_name);

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
  void setGraspPose(const Pose grasp_pose);

  /*!
   * \brief Serialized point cloud buffer accessor.
   *
   * Get the buffer containing the serialized point cloud message buffer.
   *
   * \return The buffer containing the serialized point cloud message buffer.
   */
  const uint8_t *getPointCloud() const;

  /*!
   * \brief Serialized point cloud buffer mutator.
   *
   * Set the serialized point cloud buffer to the given value. This method will copy over the entire buffer.
   *
   * \param point_cloud The serialized point cloud buffer data.
   * \param point_cloud_size The size of the serialized point cloud buffer.
   */
  void setPointCloud(const uint8_t *point_cloud, const uint32_t point_cloud_size);

  /*!
   * \brief Serialized point cloud buffer mutator.
   *
   * Set the serialized point cloud buffer to the given value based on the ROS message. This method will serialize and copy the data.
   *
   * \param point_cloud The ROS PointCloud2 messsage to serialize and store in a buffer.
   */
  void setPointCloud(const sensor_msgs::PointCloud2 &point_cloud);

  /*!
   * \brief Point cloud buffer size accessor.
   *
   * Get the size of the serialized point cloud buffer.
   *
   * \return The serialized point cloud buffer size.
   */
  uint32_t getPointCloudSize() const;

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

private:
  /*!
   * \brief Copy the point cloud buffer internally.
   *
   * Copy the serialized point cloud buffer to an internal buffer. An optional flag can be set if the old buffer should be cleaned.
   *
   * \param point_cloud The serialized ROS sensor_msgs/PointCloud2 message of the segmented object.
   * \param point_cloud_size The size of the serialized point cloud buffer.
   * \param clean If the old buffer should be cleaned (defaults to false).
   */
  void copyPointCloudBuffer(const uint8_t *point_cloud, const uint32_t point_cloud_size, const bool clean = false);

  /*!
   * \brief Serialize and copy a ROS point cloud internally.
   *
   * Serialize a ROS PointCloud2 message and copy the serialized point cloud buffer to an internal buffer. An optional flag can be set if the old buffer should be cleaned.
   *
   * \param point_cloud The ROS sensor_msgs/PointCloud2 messageto serialize and store.
   * \param clean If the old buffer should be cleaned (defaults to false).
   */
  void copyPointCloudBuffer(const sensor_msgs::PointCloud2 &point_cloud, const bool clean = false);

  /*! The ID and current size of the point cloud buffer. */
  uint32_t id_, point_cloud_size_;
  /*! The name of the object for this demonstration entry. */
  std::string object_name_;
  /*! The grasp pose data. */
  Pose grasp_pose_;
  /*! The serialized point cloud buffer. */
  uint8_t *point_cloud_;
  /*! The created timestamp. */
  time_t created_;
};

}
}
}

#endif
