uint32 id                            # The ID of the model
string object_name                   # The name of the object (non-unique)
GraspWithSuccessRate[] grasps        # The grasps associated with this model
sensor_msgs/PointCloud2 point_cloud  # The point cloud data for the model
float64 avg_color                    # The average RGB color of the point cloud
float64 std_dev_color                # The standard deviation of the RGB color of the point cloud
float64 max_distance                 # The maximum distance between any two points in the point cloud
time created                         # The time of creation for the model
