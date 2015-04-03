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
