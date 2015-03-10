cmake_minimum_required(VERSION 2.8.3)
project(rail_pick_and_place_wrapper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp actionlib message_generation rail_manipulation_msgs rail_pick_and_place_msgs rail_pick_and_place_tools rail_segmentation sensor_msgs wpi_jaco_msgs)

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
add_executable(pick_and_place src/pick_and_place.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pick_and_place
  ${catkin_LIBRARIES}
)

add_dependencies(pick_and_place rail_segmentation_generate_messages_cpp)
add_dependencies(pick_and_place rail_pick_and_place_messages_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS pick_and_place
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
