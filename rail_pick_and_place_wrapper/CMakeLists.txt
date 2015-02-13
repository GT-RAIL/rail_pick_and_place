cmake_minimum_required(VERSION 2.8.3)
project(rail_pick_and_place_wrapper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  rail_pick_and_place_msgs
  rail_recognition
  rail_segmentation
  sensor_msgs
  wpi_jaco_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rail_pick_and_place_msgs rail_recognition rail_segmentation sensor_msgs wpi_jaco_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#include_directories(/usr/include/vtk-5.8)

add_executable(pick_and_place src/pick_and_place.cpp)

add_dependencies(pick_and_place rail_segmentation_generate_messages_cpp)

#target_link_libraries(pc_registration ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so libmysqlcppconn.so)
target_link_libraries(pick_and_place ${catkin_LIBRARIES})
