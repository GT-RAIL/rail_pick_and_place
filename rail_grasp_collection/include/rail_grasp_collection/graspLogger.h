#include <ros/ros.h>
#include <rail_grasp_collection/PickupActionResult.h>

#include <iostream>
#include <fstream>
#include <sstream>

class graspLogger
{
public:
  ros::NodeHandle n;

  int graspNum;

  ros::Subscriber graspSubscriber;

  void graspCallback(const rail_grasp_collection::PickupActionResult pickupResult);

  /**
  * Constructor
  */
  graspLogger();
};
