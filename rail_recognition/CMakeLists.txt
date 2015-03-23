cmake_minimum_required(VERSION 2.8.3)
project(rail_recognition)

find_package(catkin REQUIRED COMPONENTS roscpp roslib actionlib actionlib_msgs graspdb pcl_conversions pcl_ros rail_manipulation_msgs message_generation rail_grasping rail_pick_and_place_msgs rail_segmentation sensor_msgs std_msgs std_srvs)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DisplayModel.srv
  GetModelNumbers.srv
  ReadGrasp.srv
  Release.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  GenerateModels.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  rail_grasping
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
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## Declare a cpp executable
add_executable(pc_registration src/PCRegistration.cpp)
add_executable(pc_recognition src/PCRecognition.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pc_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so)
target_link_libraries(pc_recognition ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so)

add_dependencies(pc_registration ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pc_recognition ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS pc_registration pc_recognition
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
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

## Copy model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
