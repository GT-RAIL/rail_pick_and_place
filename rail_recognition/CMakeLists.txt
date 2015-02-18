cmake_minimum_required(VERSION 2.8.3)
project(rail_recognition)

find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs std_srvs pcl_conversions pcl_ros manipulation_msgs message_generation rail_grasping rail_pick_and_place_msgs rail_segmentation sensor_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ReadGrasp.srv
  Release.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
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
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(grasp_reader src/GraspReader.cpp)
add_executable(pc_registration src/pcRegistration.cpp)
add_executable(pc_saver src/pcSaver.cpp)
add_executable(pc_recognition src/pcRecognition.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pc_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so)
target_link_libraries(pc_recognition ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so)
target_link_libraries(grasp_reader ${catkin_LIBRARIES})
target_link_libraries(pc_saver ${catkin_LIBRARIES})

add_dependencies(grasp_reader ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pc_registration ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pc_saver ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pc_recognition ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS grasp_reader pc_registration pc_saver pc_recognition
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