#include <rail_recognition/ObjectRecognitionListener.h>
#include <rail_recognition/PCRecognizer.h>

using namespace std;
using namespace rail::pick_and_place;

ObjectRecognitionListener::ObjectRecognitionListener() : private_node_("~")
{
  // set defaults
  string segmented_objects_topic("/segmentation/segmented_objects");
  int port = graspdb::Client::DEFAULT_PORT;
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");

  // grab any parameters we need
  private_node_.getParam("segmented_objects_topic", segmented_objects_topic);
  node_.getParam("/graspdb/host", host);
  node_.getParam("/graspdb/port", port);
  node_.getParam("/graspdb/user", user);
  node_.getParam("/graspdb/password", password);
  node_.getParam("/graspdb/db", db);

  // connect to the grasp database
  graspdb_ = new graspdb::Client(host, port, user, password, db);
  okay_ = graspdb_->connect();

  segmented_objects_sub_ = node_.subscribe(segmented_objects_topic, 1,
      &ObjectRecognitionListener::segmentedObjectsCallback, this);
  recognized_objects_pub_ = private_node_.advertise<rail_manipulation_msgs::SegmentedObjectList>(
      "recognized_objects", 1);


  if (okay_)
  {
    ROS_INFO("Object Recognition Listener Successfully Initialized");
  }
}

ObjectRecognitionListener::~ObjectRecognitionListener()
{
  // cleanup
  graspdb_->disconnect();
  delete graspdb_;
}

bool ObjectRecognitionListener::okay() const
{
  return okay_;
}

void ObjectRecognitionListener::segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &msg)
{
  ROS_INFO("Received %li segmented objects.", msg.objects.size());

  // check against the old list to prevent throwing out data
  rail_manipulation_msgs::SegmentedObjectList new_list;
  for (size_t i = 0; i < msg.objects.size(); i++)
  {
    bool matched = false;
    // search for a match on the point cloud
    for (size_t j = 0; j < object_list_.objects.size(); j++)
    {
      // only do a compare if we previously recognized the object
      if (object_list_.objects[j].recognized &&
          this->comparePointClouds(msg.objects[i].point_cloud, object_list_.objects[j].point_cloud))
      {
        ROS_INFO("Found a match from previously recognized objects.");
        matched = true;
        new_list.objects.push_back(object_list_.objects[j]);
        break;
      }
    }

    // check if we didn't match
    if (!matched)
    {
      new_list.objects.push_back(msg.objects[i]);
    }
  }

  // store the list
  object_list_ = new_list;

  // run recognition
  ROS_INFO("Running recognition...");
  // populate candidates
  vector<graspdb::GraspModel> candidates;
  graspdb_->loadGraspModels(candidates);
  // go through the current list
  PCRecognizer recognizer;
  for (size_t i = 0; i < object_list_.objects.size(); i++)
  {
    // check if it is already recognized
    rail_manipulation_msgs::SegmentedObject &object = object_list_.objects[i];
    if (!object.recognized)
    {
      // perform recognition
      recognizer.recognizeObject(&object, candidates);
    }
  }

  // republish the new list
  recognized_objects_pub_.publish(object_list_);
  ROS_INFO("New recognized objects published.");
}

bool ObjectRecognitionListener::comparePointClouds(const sensor_msgs::PointCloud2 &pc1,
    const sensor_msgs::PointCloud2 &pc2) const
{
  // check the size first
  if (pc1.data.size() != pc2.data.size())
  {
    return false;
  }

  // check each data point
  for (size_t i = 0; i < pc1.data.size(); i++)
  {
    if (pc1.data[i] != pc2.data[i])
    {
      // break early
      return false;
    }
  }

  // all checks passed
  return true;
}
