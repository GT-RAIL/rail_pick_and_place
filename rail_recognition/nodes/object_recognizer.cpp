/*!
 * \file object_recognizer.cpp
 * \brief The object recognizer node.
 *
 * The object recognizer sets up an action server that allows the recognition of a single segmented object.
 *
 * \author David Kent, WPI - rctoris@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

#include "rail_recognition/ObjectRecognizer.h"

using namespace std;
using namespace rail::pick_and_place;

/*!
 * Creates and runs the object_recognizer node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "object_recognizer");
  ObjectRecognizer recognizer;
  // check if everything started okay
  if (recognizer.okay())
  {
    ros::spin();
    return EXIT_SUCCESS;
  } else
  {
    return EXIT_FAILURE;
  }
}
