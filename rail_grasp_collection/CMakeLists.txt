cmake_minimum_required(VERSION 2.8.3)
project(rail_grasp_collection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  graspdb
  rail_manipulation_msgs
  rail_pick_and_place_msgs
  roscpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)

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
  ${boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(rail_grasp_collection
  nodes/rail_grasp_collection.cpp
  src/GraspCollector.cpp
)
add_executable(rail_grasp_retriever
  nodes/rail_grasp_retriever.cpp
  src/GraspRetriever.cpp
)

## Add message build dependencies (needed for source build)
add_dependencies(rail_grasp_collection
  rail_pick_and_place_msgs_gencpp
)
add_dependencies(rail_grasp_retriever
  rail_pick_and_place_msgs_gencpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(rail_grasp_collection
  ${boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(rail_grasp_retriever
  ${boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS rail_grasp_collection rail_grasp_retriever
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
