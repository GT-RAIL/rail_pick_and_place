/*!
 * \file model_generator.cpp
 * \brief The grasp model generator node.
 *
 * The grasp model generator allows for generating graspdb models based on registration criteria. An action server is
 * used to provide the model/grasp demonstration IDs to use during registration.
 *
 * \author David Kent, WPI - rctoris@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 6, 2015
 */

#include "rail_recognition/ModelGenerator.h"

using namespace std;
using namespace rail::pick_and_place;

/*!
 * Creates and runs the model_generator node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "model_generator");
  ModelGenerator generator;
  // check if everything started okay
  if (generator.okay())
  {
    ros::spin();
    return EXIT_SUCCESS;
  } else
  {
    return EXIT_FAILURE;
  }
}
