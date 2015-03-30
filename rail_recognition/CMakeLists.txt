cmake_minimum_required(VERSION 2.8.3)
project(rail_recognition)

find_package(catkin REQUIRED COMPONENTS roscpp roslib actionlib actionlib_msgs graspdb pcl_conversions pcl_ros rail_manipulation_msgs rail_pick_and_place_msgs rail_segmentation sensor_msgs std_msgs std_srvs)

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
add_executable(model_generator src/ModelGenerator.cpp)
add_executable(object_recognizer src/ObjectRecognizer.cpp src/PCRecognizer.cpp)
add_executable(object_recognition_listener
  nodes/object_recognition_listener.cpp
  src/ObjectRecognitionListener.cpp
  src/PCRecognizer.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(model_generator ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so)
target_link_libraries(object_recognizer ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so)
target_link_libraries(object_recognition_listener ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkGraphics.so)

add_dependencies(model_generator rail_pick_and_place_msgs_generate_messages_cpp rail_manipulation_msgs_generate_messages_cpp)
add_dependencies(object_recognizer rail_manipulation_msgs_generate_messages_cpp)
add_dependencies(object_recognition_listener rail_manipulation_msgs_generate_messages_cpp)

#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS model_generator object_recognizer object_recognition_listener
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
