/*!
 * \file ObjectRecognitionListener.cpp
 * \brief The object recognition listener node object.
 *
 * The object recognition listener will listen to a specified SegmentedObjectsArray topic and attempt to recognize
 * all segmented objects. The new list are republished on a separate topic.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

// RAIL Recognition
#include "rail_recognition/ObjectRecognitionListener.h"
#include "rail_recognition/PointCloudRecognizer.h"

// ROS
#include <geometry_msgs/PoseArray.h>

using namespace std;
using namespace rail::pick_and_place;

ObjectRecognitionListener::ObjectRecognitionListener() : private_node_("~")
{
  // set defaults
  debug_ = DEFAULT_DEBUG;
  string segmented_objects_topic("/segmentation/segmented_objects");
  int port = graspdb::Client::DEFAULT_PORT;
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");

  // grab any parameters we need
  private_node_.getParam("debug", debug_);
  private_node_.getParam("segmented_objects_topic", segmented_objects_topic);
  node_.getParam("/graspdb/host", host);
  node_.getParam("/graspdb/port", port);
  node_.getParam("/graspdb/user", user);
  node_.getParam("/graspdb/password", password);
  node_.getParam("/graspdb/db", db);

  // connect to the grasp database
  graspdb_ = new graspdb::Client(host, port, user, password, db);
  okay_ = graspdb_->connect();

  // setup a debug publisher if we need it
  if (debug_)
  {
    debug_pub_ = private_node_.advertise<geometry_msgs::PoseArray>("debug", 1, true);
  }

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

void ObjectRecognitionListener::segmentedObjectsCallback(
    const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &objects)
{
  ROS_INFO("Received %li segmented objects.", objects->objects.size());

  // check against the old list to prevent throwing out data
  vector<rail_manipulation_msgs::SegmentedObject> new_list;
  for (size_t i = 0; i < objects->objects.size(); i++)
  {
    bool matched = false;
    // search for a match on the point cloud
    for (size_t j = 0; j < object_list_.objects.size(); j++)
    {
      // only do a compare if we previously recognized the object
      if (object_list_.objects[j].recognized &&
          this->comparePointClouds(objects->objects[i].point_cloud, object_list_.objects[j].point_cloud))
      {
        ROS_INFO("Found a match from previously recognized objects.");
        matched = true;
        new_list.push_back(object_list_.objects[j]);
        break;
      }
    }

    // check if we didn't match
    if (!matched)
    {
      new_list.push_back(objects->objects[i]);
    }
  }

  // store the list
  object_list_.objects = new_list;

  // run recognition
  ROS_INFO("Running recognition...");
  // populate candidates
  vector<graspdb::GraspModel> candidates;
  graspdb_->loadGraspModels(candidates);
  // convert to PCL grasp models
  vector<PCLGraspModel> pcl_candidates;
  for (size_t i = 0; i < candidates.size(); i++)
  {
    pcl_candidates.push_back(PCLGraspModel(candidates[i]));
  }

  // go through the current list
  PointCloudRecognizer recognizer;
  for (size_t i = 0; i < object_list_.objects.size(); i++)
  {
    // check if it is already recognized
    rail_manipulation_msgs::SegmentedObject &object = object_list_.objects[i];
    if (!object.recognized)
    {
      // perform recognition
      recognizer.recognizeObject(object, pcl_candidates);
    }
  }

  // republish the new list
  recognized_objects_pub_.publish(object_list_);
  // check for debug publishing
  if (debug_)
  {
    geometry_msgs::PoseArray poses;
    for (size_t i = 0; i < object_list_.objects.size(); i++)
    {
      for (size_t j = 0; j < object_list_.objects[i].grasps.size(); j++)
      {
        poses.header = object_list_.objects[i].grasps[j].header;
        poses.poses.push_back(object_list_.objects[i].grasps[j].pose);
      }
    }
    debug_pub_.publish(poses);
  }

  ROS_INFO("New recognized objects published.");
}

bool ObjectRecognitionListener::comparePointClouds(const sensor_msgs::PointCloud2 &pc1,
    const sensor_msgs::PointCloud2 &pc2) const
{
  // check the size first then compare
  return (pc1.data.size() == pc2.data.size()) && (pc1.data == pc1.data);
}
