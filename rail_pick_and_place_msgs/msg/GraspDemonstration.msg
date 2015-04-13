uint32 id                             # The ID of the grasp demonstration
string object_name                    # The name of the object that was grasped
geometry_msgs/PoseStamped grasp_pose  # The pose of the grasp in reference to the robot's fixed frame
string eef_frame_id                   # The name of the robot's end effector frame that this grasp was collected from
sensor_msgs/PointCloud2 point_cloud   # The segmented object point cloud
sensor_msgs/Image image               # The segmented object image
time created                          # The time of creation for the grasp demonstration
