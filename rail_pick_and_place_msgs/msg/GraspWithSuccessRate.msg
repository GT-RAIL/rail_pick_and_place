geometry_msgs/PoseStamped grasp_pose  # The grasp pose in reference to the robot's fixed frame
string eef_frame_id                   # The name of the robot's end effector frame that this grasp was collected from
uint32 successes                      # The number of recorded times this grasp was successful (useful for success rate)
uint32 attempts                       # The number of recorded times this grasp was attempted (useful for success rate)
