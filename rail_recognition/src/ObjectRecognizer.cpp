/*!
 * \file ObjectRecognizer.cpp
 * \brief The object recognizer node object.
 *
 * The object recognizer sets up an action server that allows the recognition of a single segmented object.
 *
 * \author David Kent, WPI - rctoris@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

// RAIL Recognition
#include "rail_recognition/ObjectRecognizer.h"
#include "rail_recognition/PointCloudRecognizer.h"

using namespace std;
using namespace rail::pick_and_place;

ObjectRecognizer::ObjectRecognizer()
    : private_node_("~"),
      as_(private_node_, "recognize_object", boost::bind(&ObjectRecognizer::recognizeObjectCallback, this, _1), false)
{
  // set defaults
  int port = graspdb::Client::DEFAULT_PORT;
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");

  // grab any parameters we need
  node_.getParam("/graspdb/host", host);
  node_.getParam("/graspdb/port", port);
  node_.getParam("/graspdb/user", user);
  node_.getParam("/graspdb/password", password);
  node_.getParam("/graspdb/db", db);

  // connect to the grasp database
  graspdb_ = new graspdb::Client(host, port, user, password, db);
  okay_ = graspdb_->connect();

  // start the action server
  as_.start();

  if (okay_)
  {
    ROS_INFO("Object Recognizer Successfully Initialized");
  }
}

ObjectRecognizer::~ObjectRecognizer()
{
  // cleanup
  as_.shutdown();
  graspdb_->disconnect();
  delete graspdb_;
}

bool ObjectRecognizer::okay() const
{
  return okay_;
}

void ObjectRecognizer::recognizeObjectCallback(const rail_manipulation_msgs::RecognizeObjectGoalConstPtr &goal)
{
  ROS_INFO("Recognize Object Request Received.");

  rail_manipulation_msgs::RecognizeObjectFeedback feedback;
  feedback.message = "Loading candidate models...";
  as_.publishFeedback(feedback);

  // populate candidates based on the name if it exists
  vector<graspdb::GraspModel> candidates;
  if (goal->name.size() > 0)
  {
    graspdb_->loadGraspModelsByObjectName(goal->name, candidates);
  }
  else
  {
    graspdb_->loadGraspModels(candidates);
  }

  // convert to PCL grasp models
  vector<PCLGraspModel> pcl_candidates;
  for (size_t i = 0; i < candidates.size(); i++)
  {
    pcl_candidates.push_back(PCLGraspModel(candidates[i]));
  }

  // copy the information to the result
  rail_manipulation_msgs::RecognizeObjectResult result;
  result.object = goal->object;

  // perform recognition
  feedback.message = "Running recognition...";
  as_.publishFeedback(feedback);
  PointCloudRecognizer recognizer;
  if (!recognizer.recognizeObject(result.object, pcl_candidates))
  {
    as_.setSucceeded(result, "Object could not be recognized.");
  }
  else
  {
    as_.setSucceeded(result, "Object successfully recognized.");
  }
}
