/*!
 * \file GraspModelRetriever.cpp
 * \brief The grasp model retriever node object.
 *
 * The grasp model retriever allows for loading stored models from the grasp database training set. An action server is
 * started as the main entry point to grasp retrieval. A latched topic is used to publish the resulting point cloud and
 * pose array.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 3, 2015
 */

// RAIL Grasp Collection
#include "rail_recognition/GraspModelRetriever.h"

// ROS
#include <geometry_msgs/PoseArray.h>

using namespace std;
using namespace rail::pick_and_place;

GraspModelRetriever::GraspModelRetriever()
    : private_node_("~"),
      as_(private_node_, "retrieve_grasp_model", boost::bind(&GraspModelRetriever::retrieveGraspModel, this, _1), false)
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

  // set up a connection to the grasp database
  graspdb_ = new graspdb::Client(host, port, user, password, db);
  okay_ = graspdb_->connect();

  // set up the latched publishers we need
  point_cloud_pub_ = private_node_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1, true);
  poses_pub_ = private_node_.advertise<geometry_msgs::PoseArray>("poses", 1, true);

  // start the action server
  as_.start();

  if (okay_)
  {
    ROS_INFO("Grasp Model Retriever Successfully Initialized");
  }
}

GraspModelRetriever::~GraspModelRetriever()
{
  // cleanup
  as_.shutdown();
  graspdb_->disconnect();
  delete graspdb_;
}

bool GraspModelRetriever::okay() const
{
  return okay_;
}

void GraspModelRetriever::retrieveGraspModel(const rail_pick_and_place_msgs::RetrieveGraspModelGoalConstPtr &goal)
{
  rail_pick_and_place_msgs::RetrieveGraspModelFeedback feedback;
  rail_pick_and_place_msgs::RetrieveGraspModelResult result;

  // attempt to load the grasp model from the database
  feedback.message = "Requesting grasp model from database...";
  as_.publishFeedback(feedback);
  graspdb::GraspModel gm;
  if (!graspdb_->loadGraspModel(goal->id, gm))
  {
    result.success = false;
    as_.setSucceeded(result, "Could not load grasp model from database.");
    return;
  } else
  {
    // store inside of the result
    result.grasp_model = gm.toROSGraspModelMessage();

    // publish the data
    feedback.message = "Publishing to latched topics...";
    as_.publishFeedback(feedback);
    // send the resulting point cloud message from the goal
    point_cloud_pub_.publish(result.grasp_model.point_cloud);
    // create and send a PoseArray
    geometry_msgs::PoseArray poses;
    for (size_t i = 0; i < result.grasp_model.grasps.size(); i++)
    {
      poses.header.frame_id = result.grasp_model.grasps[i].grasp_pose.header.frame_id;
      poses.poses.push_back(result.grasp_model.grasps[i].grasp_pose.pose);
    }
    poses_pub_.publish(poses);

    // success
    result.success = true;
    as_.setSucceeded(result, "Success!");
  }
}
