/*!
 * \file GraspRetriever.cpp
 * \brief The grasp retriever node object.
 *
 * The grasp retriever allows for loading stored grasps from the grasp database training set. An action server is
 * started as the main entry point to grasp retrieval. A latched topic is used to publish the resulting point cloud and
 * pose.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 4, 2015
 */

// RAIL Grasp Collection
#include "rail_grasp_collection/GraspRetriever.h"

using namespace std;
using namespace rail::pick_and_place;

GraspRetriever::GraspRetriever()
    : private_node_("~"),
      as_(private_node_, "retrieve_grasp", boost::bind(&GraspRetriever::retrieveGrasp, this, _1), false)
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
  pose_pub_ = private_node_.advertise<geometry_msgs::PoseStamped>("pose", 1, true);

  // start the action server
  as_.start();

  if (okay_)
  {
    ROS_INFO("Grasp Retriever Successfully Initialized");
  }
}

GraspRetriever::~GraspRetriever()
{
  // cleanup
  as_.shutdown();
  graspdb_->disconnect();
  delete graspdb_;
}

bool GraspRetriever::okay() const
{
  return okay_;
}

void GraspRetriever::retrieveGrasp(const rail_pick_and_place_msgs::RetrieveGraspDemonstrationGoalConstPtr &goal)
{
  rail_pick_and_place_msgs::RetrieveGraspDemonstrationFeedback feedback;
  rail_pick_and_place_msgs::RetrieveGraspDemonstrationResult result;

  // attempt to load the grasp from the database
  feedback.message = "Requesting grasp demonstration from database...";
  as_.publishFeedback(feedback);
  graspdb::GraspDemonstration gd;
  if (!graspdb_->loadGraspDemonstration(goal->id, gd))
  {
    result.success = false;
    as_.setSucceeded(result, "Could not load grasp from database.");
    return;
  } else
  {
    // store inside of the result
    result.grasp = gd.toROSGraspDemonstrationMessage();

    // publish the data
    feedback.message = "Publishing to latched topics...";
    as_.publishFeedback(feedback);
    // send the resulting point cloud message from the goal
    point_cloud_pub_.publish(result.grasp.point_cloud);
    // create and send a PoseStamped
    pose_pub_.publish(gd.getGraspPose().toROSPoseStampedMessage());

    // success
    result.success = true;
    as_.setSucceeded(result, "Success!");
  }
}
