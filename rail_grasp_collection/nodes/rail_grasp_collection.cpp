/*!
 * \file rail_grasp_collection.cpp
 * \brief The main grasp collector node.
 *
 * The grasp collector is responsible for capturing and storing grasps. An action server is started is the main
 * entry point to grasp collecting.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

#include "rail_grasp_collection/GraspCollector.h"

using namespace std;
using namespace rail::pick_and_place;

/*!
 * Creates and runs the rail_grasp_collection node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rail_grasp_collection");
  GraspCollector collector;
  // check if everything started okay
  if (collector.okay())
  {
    ros::spin();
    return EXIT_SUCCESS;
  } else
  {
    return EXIT_FAILURE;
  }
}
