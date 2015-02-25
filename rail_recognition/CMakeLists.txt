cmake_minimum_required(VERSION 2.8.3)
project(rail_recognition)

find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs std_srvs pcl_conversions pcl_ros manipulation_msgs message_generation rail_grasping rail_pick_and_place_msgs rail_segmentation sensor_msgs rviz)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DisplayModel.srv
  GenerateModels.srv
  GetModelNumbers.srv
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
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/rail_recognition/ModelGenerationPanel.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/ModelGenerationPanel.cpp
  ${MOC_FILES}
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
add_library(${PROJECT_NAME} ${SOURCE_FILES})

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(grasp_reader src/GraspReader.cpp)
add_executable(pc_registration src/PCRegistration.cpp)
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