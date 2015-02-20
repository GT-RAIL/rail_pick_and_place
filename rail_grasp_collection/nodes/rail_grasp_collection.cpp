#include <rail_grasp_collection/GraspCollector.h>
#include <graspdb/Client.h>

using namespace std;
using namespace rail::pick_and_place;

int main(int argc, char **argv)
{
  graspdb::Client client("127.0.0.1", 5432, "ros", "testpassword", "graspdb");
  cout << client.connected() << endl;
  cout << client.connect() << endl;
  cout << client.connected() << endl;

  // initialize ROS and the node
  ros::init(argc, argv, "rail_grasp_collection");
  GraspCollector collector;
  ros::spin();

  return EXIT_SUCCESS;
}
