cmake_minimum_required(VERSION 2.8.3)
project(rail_recognition)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  geometry_msgs
  graspdb
  pcl_conversions
  pcl_ros
  rail_manipulation_msgs
  rail_segmentation
  rail_pick_and_place_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
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
add_executable(metric_trainer
  nodes/metric_trainer.cpp
  src/MetricTrainer.cpp
  src/PointCloudMetrics.cpp
)
add_executable(model_generator
  nodes/model_generator.cpp
  src/ModelGenerator.cpp
  src/PCLGraspModel.cpp
  src/PointCloudMetrics.cpp
)
add_executable(object_recognizer
  nodes/object_recognizer.cpp
  src/ObjectRecognizer.cpp
  src/PCLGraspModel.cpp
  src/PointCloudMetrics.cpp
  src/PointCloudRecognizer.cpp
)
add_executable(object_recognition_listener
  nodes/object_recognition_listener.cpp
  src/ObjectRecognitionListener.cpp
  src/PCLGraspModel.cpp
  src/PointCloudMetrics.cpp
  src/PointCloudRecognizer.cpp
  src/ImageRecognizer.cpp
)
add_executable(rail_grasp_model_retriever
  nodes/rail_grasp_model_retriever.cpp
  src/GraspModelRetriever.cpp
)
add_executable(train_image_recognizer
  nodes/train_image_recognizer.cpp
  src/ImageRecognizer.cpp
)
add_executable(test_image_recognizer
  nodes/test_image_recognizer.cpp
  src/ImageRecognizer.cpp
)
add_executable(collect_images
  nodes/collect_images.cpp
  src/ImageRecognizer.cpp
)
add_executable(save_image_recognizer_features
  nodes/save_image_recognizer_features.cpp
  src/ImageRecognizer.cpp
)

## Add message build dependencies (needed for source build)
add_dependencies(metric_trainer
  rail_pick_and_place_msgs_generate_messages_cpp
)
add_dependencies(model_generator
  rail_pick_and_place_msgs_generate_messages_cpp
)
add_dependencies(object_recognizer
  rail_manipulation_msgs_generate_messages_cpp
)
add_dependencies(object_recognition_listener
  rail_manipulation_msgs_generate_messages_cpp
)
add_dependencies(rail_grasp_model_retriever
  rail_pick_and_place_msgs_generate_messages_cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(metric_trainer
  ${catkin_LIBRARIES}
)
target_link_libraries(model_generator
  ${catkin_LIBRARIES}
)
target_link_libraries(object_recognizer
 ${catkin_LIBRARIES}
)
target_link_libraries(object_recognition_listener
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
 opencv_nonfree
)
target_link_libraries(rail_grasp_model_retriever
 ${catkin_LIBRARIES}
)
target_link_libraries(train_image_recognizer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  opencv_nonfree
)
target_link_libraries(test_image_recognizer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  opencv_nonfree
)
target_link_libraries(collect_images
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  opencv_nonfree
)
target_link_libraries(save_image_recognizer_features
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  opencv_nonfree
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  collect_images
  metric_trainer
  model_generator
  object_recognizer
  object_recognition_listener
  rail_grasp_model_retriever
  save_image_recognizer_features
  train_image_recognizer
  test_image_recognizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy data files
install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
