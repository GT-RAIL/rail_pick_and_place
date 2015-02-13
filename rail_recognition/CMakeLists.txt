cmake_minimum_required(VERSION 2.8.3)
project(rail_recognition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  std_srvs
  pcl_conversions
  pcl_ros
  manipulation_msgs
  message_generation
  rail_grasping
  rail_pick_and_place_msgs
  rail_segmentation
  sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  ReadGrasp.srv
  Release.srv
)

generate_messages(DEPENDENCIES rail_grasping sensor_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib std_msgs std_srvs pcl_conversions pcl_ros manipulation_msgs rail_grasping rail_pick_and_place_msgs rail_segmentation sensor_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#include_directories(/usr/include/vtk-5.8)

add_executable(grasp_reader src/GraspReader.cpp)
add_executable(pc_registration src/pcRegistration.cpp)
add_executable(pc_saver src/pcSaver.cpp)
add_executable(pc_recognition src/pcRecognition.cpp)

add_dependencies(grasp_reader ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pc_registration ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pc_saver ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pc_recognition ${PROJECT_NAME}_generate_messages_cpp)

#target_link_libraries(pc_registration ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so libmysqlcppconn.so)
target_link_libraries(pc_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so)
target_link_libraries(pc_recognition ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so)
target_link_libraries(grasp_reader ${catkin_LIBRARIES})
target_link_libraries(pc_saver ${catkin_LIBRARIES})
