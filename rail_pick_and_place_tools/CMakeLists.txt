cmake_minimum_required(VERSION 2.8.3)
project(rail_pick_and_place_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  graspdb
  rail_pick_and_place_msgs
  rviz
  roscpp
  std_srvs
)
find_package(Boost REQUIRED COMPONENTS thread)

## This plugin includes Qt widgets, so we must include Qt
find_package(Qt4 REQUIRED COMPONENTS
  QtCore
  QtGui
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
  ${boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Include for use with Qt
include(${QT_USE_FILE})

## For use with Qt signals and slots
add_definitions(-DQT_NO_KEYWORDS)

## Specify which header files need to be run through "moc" (Qt's meta-object compiler).
qt4_wrap_cpp(MOC_FILES
  include/rail_pick_and_place_tools/GraspCollectionPanel.h
  include/rail_pick_and_place_tools/MetricTrainingPanel.h
  include/rail_pick_and_place_tools/ModelGenerationPanel.h
  include/rail_pick_and_place_tools/SegmentPanel.h
)

## Specify plugin source files
set(SOURCE_FILES
  src/GraspCollectionPanel.cpp
  src/MetricTrainingPanel.cpp
  src/ModelGenerationPanel.cpp
  src/SegmentPanel.cpp
  ${MOC_FILES}
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCE_FILES})

## Add message build dependencies (needed for source build)
add_dependencies(${PROJECT_NAME}
  rail_pick_and_place_msgs_generate_msgs_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
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

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Copy rviz files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

## Copy splash screen files
install(DIRECTORY images/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/images
  FILES_MATCHING PATTERN "*.png"
)

## Copy plugin config files
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
