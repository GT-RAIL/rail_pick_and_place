cmake_minimum_required(VERSION 2.8.3)
project(rail_grasp_collection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  actionlib_msgs
  wpi_jaco_msgs
  pcl_conversions
  pcl_ros
  rail_segmentation
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  ClosestObject.srv
  Segment.srv
)

add_action_files(
  FILES
  Pickup.action
)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp actionlib actionlib_msgs wpi_jaco_msgs pcl_conversions pcl_ros rail_segmentation sensor_msgs std_msgs tf trajectory_msgs geometry_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rail_grasp_collection src/graspCollection.cpp)
add_dependencies(rail_grasp_collection ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rail_grasp_collection ${catkin_LIBRARIES})

add_executable(object_tracker src/objectTracker.cpp)
add_dependencies(object_tracker ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(object_tracker ${catkin_LIBRARIES})

add_executable(grasp_logger src/graspLogger.cpp)
add_dependencies(grasp_logger ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(grasp_logger ${catkin_LIBRARIES})

