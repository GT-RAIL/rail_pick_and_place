/*!
 * \file metric_trainer.cpp
 * \brief The metric trainer node.
 *
 * The metric trainer allows for generating data sets for training registration metric decision trees. An action server
 * is used to provide the object name and files are dumped to "registration_metrics.txt".
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

#include "rail_recognition/MetricTrainer.h"

using namespace std;
using namespace rail::pick_and_place;

/*!
 * Creates and runs the metric_trainer node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "metric_trainer");
  MetricTrainer mt;
  // check if everything started okay
  if (mt.okay())
  {
    ros::spin();
    return EXIT_SUCCESS;
  } else
  {
    return EXIT_FAILURE;
  }
}
