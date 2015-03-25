#include <rail_recognition/ObjectRecognitionListener.h>

using namespace std;
using namespace pcl;
using namespace rail::pick_and_place;

ObjectRecognitionListener::ObjectRecognitionListener() :
    asRecognize(n, "rail_recognition/recognize", boost::bind(&ObjectRecognitionListener::executeRecognize, this, _1), false),
    asRecognizeAll(n, "rail_recognition/recognize_all", boost::bind(&ObjectRecognitionListener::executeRecognizeAll, this, _1), false)
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

  ros::NodeHandle pnh("~");
  string objectTopic("/rail_segmentation/segmented_objects");
  pnh.getParam("object_topic", objectTopic);
  segmentedObjectsSubscriber = n.subscribe(objectTopic, 1, &ObjectRecognitionListener::objectsCallback, this);
  recognizedObjectsPublisher = n.advertise<rail_manipulation_msgs::SegmentedObjectList>("rail_recognition/recognized_objects", 1);

  xTrans = 0.0;
  yTrans = 0.0;
  zTrans = 0.0;

  asRecognize.start();
  asRecognizeAll.start();
}

void ObjectRecognitionListener::objectsCallback(const rail_manipulation_msgs::SegmentedObjectList& msg)
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);
  objectList = msg;
  ROS_INFO("Received new segmented objects.");
}

void ObjectRecognitionListener::executeRecognize(const rail_manipulation_msgs::RecognizeGoalConstPtr &goal)
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

  rail_manipulation_msgs::RecognizeResult result;
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);
    //perform recognition
    if (goal->index < objectList.objects.size())
    {
      result.success = recognizer.recognizeObject(&objectList.objects[goal->index], candidates);
    }
    if (result.success)
    {
      recognizedObjectsPublisher.publish(objectList);
      ROS_INFO("Object successfully recognized: %s", objectList.objects[goal->index].name.c_str());
    }
    else
    {
      ROS_INFO("Object could not be recognized.");
    }
  }

  asRecognize.setSucceeded(result);
}

void ObjectRecognitionListener::executeRecognizeAll(const rail_manipulation_msgs::RecognizeAllGoalConstPtr &goal)
{
  rail_manipulation_msgs::RecognizeAllFeedback feedback;
  stringstream ss;
  ss << "Populating candidates for recognition...";
  feedback.message == ss.str();
  asRecognizeAll.publishFeedback(feedback);

  //populate candidates
  vector<graspdb::GraspModel> candidates;
  vector<string> names;
  graspdb->getUniqueGraspModelObjectNames(names);
  for (unsigned int i = 0; i < names.size(); i ++)
  {
    vector<graspdb::GraspModel> tempCandidates;
    graspdb->loadGraspModelsByObjectName(names[i], tempCandidates);
    candidates.insert(candidates.end(), tempCandidates.begin(), tempCandidates.end());
  }

  rail_manipulation_msgs::RecognizeAllResult result;
  {
    boost::recursive_mutex::scoped_lock lock(api_mutex);

    ss.str("");
    ss << "Beginning recognition for " << objectList.objects.size() << " objects...";
    feedback.message == ss.str();
    asRecognizeAll.publishFeedback(feedback);
    result.successes.resize(objectList.objects.size());

    bool recognizedSomething = false;
    for (unsigned int i = 0; i < objectList.objects.size(); i++)
    {

      //perform recognition
      result.successes[i] = recognizer.recognizeObject(&objectList.objects[i], candidates);
      if (result.successes[i])
      {
        recognizedObjectsPublisher.publish(objectList);
        ROS_INFO("Object %d successfully recognized: %s", i, objectList.objects[i].name.c_str());

        if (!recognizedSomething)
          recognizedSomething = true;

        ss.str("");
        ss << "Successfully recognized object " << i << " as " << objectList.objects[i].name << ", " << objectList.objects.size() - i - 1 << " objects left to recgonize...";
        feedback.message == ss.str();
        asRecognizeAll.publishFeedback(feedback);
      }
      else
      {
        ROS_INFO("Object %d could not be recognized.", i);

        ss.str("");
        ss << "Could not recognize object " << i << ", " << objectList.objects.size() - i - 1 << " objects left to recgonize...";
        feedback.message == ss.str();
        asRecognizeAll.publishFeedback(feedback);
      }
    }

    if (recognizedSomething)
    {
      ss.str("");
      ss << "Successfully recognized at least one object.";
      feedback.message == ss.str();
      asRecognizeAll.publishFeedback(feedback);

      recognizedObjectsPublisher.publish(objectList);
    }
    else
    {
      ss.str("");
      ss << "Failed to recognize any objects.";
      feedback.message == ss.str();
      asRecognizeAll.publishFeedback(feedback);
    }
  }

  asRecognizeAll.setSucceeded(result);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_recognition_listener");

  ObjectRecognitionListener orl;

  ros::spin();

  return 0;
}
