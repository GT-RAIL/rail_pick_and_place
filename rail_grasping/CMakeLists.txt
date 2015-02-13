cmake_minimum_required(VERSION 2.8.3)
project(rail_grasping)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  carl_moveit
  control_msgs
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  tf
  wpi_jaco_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  GraspModel.msg
)

add_service_files(
  FILES
  RequestGrasp.srv
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib carl_moveit control_msgs geometry_msgs message_runtime roscpp sensor_msgs tf wpi_jaco_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(grasp_object src/graspObject.cpp)
add_dependencies(grasp_object ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(grasp_object ${catkin_LIBRARIES})
