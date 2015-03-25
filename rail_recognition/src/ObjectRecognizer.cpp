#include <rail_recognition/ObjectRecognizer.h>

using namespace std;
using namespace pcl;
using namespace rail::pick_and_place;

ObjectRecognizer::ObjectRecognizer() :
    asRecognizeObject(n, "rail_recognition/recognize_object", boost::bind(&ObjectRecognizer::executeRecognizeObject, this, _1), false)
{
  //setup connection to grasp database
  // set defaults
  int port = graspdb::Client::DEFAULT_PORT;
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");

  // grab any parameters we need
  n.getParam("/graspdb/host", host);
  n.getParam("/graspdb/port", port);
  n.getParam("/graspdb/user", user);
  n.getParam("/graspdb/password", password);
  n.getParam("/graspdb/db", db);

  // connect to the grasp database
  graspdb = new graspdb::Client(host, port, user, password, db);
  bool okay = graspdb->connect();

  if (okay)
    ROS_INFO("Successfully connected to grasp database.");
  else
    ROS_INFO("Could not connect to grasp database.");

  xTrans = 0.0;
  yTrans = 0.0;
  zTrans = 0.0;

  asRecognizeObject.start();
}

void ObjectRecognizer::executeRecognizeObject(const rail_manipulation_msgs::RecognizeObjectGoalConstPtr &goal)
{
  //populate candidates
  vector<graspdb::GraspModel> candidates;
  if (goal->name.size() > 0)
  {
    graspdb->loadGraspModelsByObjectName(goal->name, candidates);
  }
  else
  {
    vector<string> names;
    graspdb->getUniqueGraspModelObjectNames(names);
    for (unsigned int i = 0; i < names.size(); i ++)
    {
      vector<graspdb::GraspModel> tempCandidates;
      graspdb->loadGraspModelsByObjectName(names[i], tempCandidates);
      candidates.insert(candidates.end(), tempCandidates.begin(), tempCandidates.end());
    }
  }

  //perform recognition
  rail_manipulation_msgs::RecognizeObjectResult result;
  result.object = goal->object;
  if (!recognizer.recognizeObject(&result.object, candidates))
  {
    ROS_INFO("Object could not be recognized.");
  }
  else
  {
    ROS_INFO("Object successfully recognized!");
  }
  asRecognizeObject.setSucceeded(result);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_recognezer");

  ObjectRecognizer o;

  ros::spin();

  return 0;
}
