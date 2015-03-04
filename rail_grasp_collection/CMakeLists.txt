cmake_minimum_required(VERSION 2.8.3)
project(rail_grasp_collection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs pcl_conversions pcl_ros rail_segmentation sensor_msgs std_msgs tf trajectory_msgs geometry_msgs rail_manipulation_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ClosestObject.srv
  Segment.srv
)

## Generate action in the 'action' folder
add_action_files(
  FILES
  Pickup.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(rail_grasp_collection src/graspCollection.cpp)
add_executable(object_tracker src/objectTracker.cpp)
add_executable(grasp_logger src/graspLogger.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rail_grasp_collection
  ${catkin_LIBRARIES}
)
target_link_libraries(object_tracker
  ${catkin_LIBRARIES}
)
target_link_libraries(grasp_logger
  ${catkin_LIBRARIES}
)

add_dependencies(grasp_logger ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(object_tracker ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(rail_grasp_collection ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS rail_grasp_collection object_tracker grasp_logger
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
