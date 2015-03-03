#include <rail_grasp_collection/GraspCollector.h>
#include <graspdb/graspdb.h>

using namespace std;
using namespace rail::pick_and_place;

int main(int argc, char **argv)
{
  graspdb::Client client("127.0.0.1", 5432, "ros", "testpassword", "graspdb");
  cout << client.connected() << endl;
  cout << client.connect() << endl;
  cout << client.connected() << endl;

  graspdb::Pose grasp_pose("test frame ID", graspdb::Position(1, 2, 3), graspdb::Orientation(1, 2, 3, 4));
  size_t point_cloud_size = 50;
  uint8_t *point_cloud = new uint8_t[point_cloud_size];
  graspdb::GraspDemonstration demonstration("test object name", grasp_pose, point_cloud, point_cloud_size);
  client.addGraspDemonstration(demonstration);

  // initialize ROS and the node
  ros::init(argc, argv, "rail_grasp_collection");
  GraspCollector collector;
  ros::spin();

  return EXIT_SUCCESS;
}
