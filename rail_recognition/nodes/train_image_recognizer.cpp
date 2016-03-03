/*!
 * \file train_image_recognizer.cpp
 * \brief Train a neural network for the image recognizer, based on sorted training images
 *
 * \author Brian Hetherman, WPI - bhetherman@wpi.edu
 * \author David Kent, WPI - russell.toris@gmail.com
 * \date May 12, 2015
 */

#include "rail_recognition/ImageRecognizer.h"

using namespace std;

/*!
 * Creates and runs the train_image_recognizer node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "train_image_recognizer");

  ImageRecognizer ir;
  ir.trainImageRecognizer();

  return EXIT_SUCCESS;
}
