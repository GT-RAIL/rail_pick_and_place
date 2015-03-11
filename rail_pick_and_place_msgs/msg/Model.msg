uint32 id                            # The ID of the model
string object_name                   # The name of the object (non-unique)
geometry_msgs/Pose[] grasp_poses     # The grasp poses for this model
string[] grasp_frame_ids             # The name of the frame for each grasp pose
uint32[] successes                   # The number of times each grasp was successful
uint32[] attempts                    # The number of times each grasp was attempted
sensor_msgs/PointCloud2 point_cloud  # The point cloud data for the model
time created                         # The time of creation for the model
