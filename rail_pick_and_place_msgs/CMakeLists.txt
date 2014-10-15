cmake_minimum_required(VERSION 2.8.3)
project(rail_pick_and_place_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  GraspRecognized.srv
  PickupObject.srv
  PickupSegmentedObject.srv
  RecognizeAndGrasp.srv
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########


