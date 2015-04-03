/*!
 * \file rail_grasp_model_retriever.cpp
 * \brief The main grasp model retriever node.
 *
 * The grasp model retriever allows for loading stored models from the grasp database training set. An action server is
 * started as the main entry point to grasp retrieval. A latched topic is used to publish the resulting point cloud and
 * pose array.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 3, 2015
 */

#include "rail_recognition/GraspModelRetriever.h"

using namespace std;
using namespace rail::pick_and_place;

/*!
 * Creates and runs the rail_grasp_model_retriever node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rail_grasp_model_retriever");
  GraspModelRetriever retriever;
  // check if everything started okay
  if (retriever.okay())
  {
    ros::spin();
    return EXIT_SUCCESS;
  } else
  {
    return EXIT_FAILURE;
  }
}
