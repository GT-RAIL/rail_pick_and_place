/*!
 * \file rail_grasp_retriever.cpp
 * \brief The main grasp retriever node.
 *
 * The grasp retriever allows for loading stored grasps from the grasp database training set. An action server is
 * started as the main entry point to grasp retrieval. A latched topic is used to publish the resulting point cloud and
 * pose.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 4, 2015
 */

#include "rail_grasp_collection/GraspRetriever.h"

using namespace std;
using namespace rail::pick_and_place;

/*!
 * Creates and runs the rail_grasp_retriever node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rail_grasp_retriever");
  GraspRetriever retriever;
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
