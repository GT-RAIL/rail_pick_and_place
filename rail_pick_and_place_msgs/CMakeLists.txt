cmake_minimum_required(VERSION 2.8.3)
project(rail_pick_and_place_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  rail_manipulation_msgs
  sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES
  GraspDemonstration.msg
  GraspModel.msg
  GraspWithSuccessRate.msg
)

## Generate services in the 'srv' folder
add_service_files(FILES
  GraspRecognized.srv
  PickupObject.srv
  PickupSegmentedObject.srv
  RecognizeAndGrasp.srv
)

## Generate actions in the 'action' folder
add_action_files(FILES
  GraspAndStore.action
  RetrieveGraspDemonstration.action
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  rail_manipulation_msgs
  sensor_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()
