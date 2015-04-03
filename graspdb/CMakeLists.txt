cmake_minimum_required(VERSION 2.8.3)
project(graspdb)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  geometry_msgs
  rail_pick_and_place_msgs
  roscpp
  roscpp_serialization
  sensor_msgs
  tf2
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roscpp_serialization
  DEPENDS pqxx
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Source files for the library
set(GRASPDB_SOURCE
  src/Client.cpp
  src/Entity.cpp
  src/Grasp.cpp
  src/GraspDemonstration.cpp
  src/GraspModel.cpp
  src/Orientation.cpp
  src/Pose.cpp
  src/Position.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${GRASPDB_SOURCE})

## Add message build dependencies (needed for source build)
add_dependencies(${PROJECT_NAME}
  rail_pick_and_place_msgs_gencpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pqxx
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
