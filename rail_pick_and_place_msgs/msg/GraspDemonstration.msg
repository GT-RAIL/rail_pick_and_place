uint32 id                                     # The ID of the grasp demonstration
string object_name                            # The name of the object that was grasped
geometry_msgs/Pose grasp_pose_fixed_frame_id  # The fixed reference frame the grasp pose is in reference to
geometry_msgs/Pose grasp_pose_grasp_frame_id  # The name of the frame for the grasp pose
geometry_msgs/Pose grasp_pose                 # The pose of the grasp
sensor_msgs/PointCloud2 point_cloud           # The segmented object point cloud
time created                                  # The time of creation for the grasp demonstration
