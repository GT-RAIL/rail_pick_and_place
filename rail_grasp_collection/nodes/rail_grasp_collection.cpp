#include <rail_grasp_collection/GraspCollector.h>

using namespace std;
using namespace rail::pick_and_place;

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rail_grasp_collection");
  GraspCollector collector;
  ros::spin();

  return EXIT_SUCCESS;
}
