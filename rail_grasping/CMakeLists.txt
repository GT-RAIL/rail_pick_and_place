cmake_minimum_required(VERSION 2.8.3)
project(rail_grasping)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib carl_moveit control_msgs geometry_msgs message_generation roscpp sensor_msgs tf wpi_jaco_msgs)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate mesages in the 'msg' folder
add_message_files(
  FILES
  GraspModel.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  RequestGrasp.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
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
add_executable(grasp_object src/graspObject.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(grasp_object
  ${catkin_LIBRARIES}
)

add_dependencies(grasp_object ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS grasp_object
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
