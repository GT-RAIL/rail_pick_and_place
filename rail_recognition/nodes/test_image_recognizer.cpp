/*!
 * \file test_image_recognizer.cpp
 * \brief Test the image recognizer with a set of previously saved and sorted test images.
 *
 * \author Brian Hetherman, WPI - bhetherman@wpi.edu
 * \author David Kent, WPI - rctoris@wpi.edu
 * \date May 12, 2015
 */

#include "rail_recognition/ImageRecognizer.h"

using namespace std;

/*!
 * Creates and runs the test_image_recognizer node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "test_image_recognizer");

  ImageRecognizer ir;
  ir.loadImageRecognizer();
  ir.testImageRecognizer();

  return EXIT_SUCCESS;
}
