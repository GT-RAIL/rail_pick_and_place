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

#include <rail_grasp_collection/GraspRetriever.h>

using namespace std;
using namespace rail::pick_and_place;

GraspRetriever::GraspRetriever()
    : private_node_("~"), host_("127.0.0.1"), user_("ros"), password_(""), db_("graspdb"),
      as_(private_node_, "retrieve_grasp", boost::bind(&GraspRetriever::retrieveGrasp, this, _1), false)
{
  // set defaults
  port_ = graspdb::Client::DEFAULT_PORT;

  // grab any parameters we need
  private_node_.getParam("host", host_);
  private_node_.getParam("port", port_);
  private_node_.getParam("user", user_);
  private_node_.getParam("password", password_);
  private_node_.getParam("db", db_);

  // set up a connection to the grasp database
  graspdb_ = new graspdb::Client(host_, port_, user_, password_, db_);
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
  ROS_INFO("Retrieve grasp requset received.");

  rail_pick_and_place_msgs::RetrieveGraspDemonstrationFeedback feedback;
  rail_pick_and_place_msgs::RetrieveGraspDemonstrationResult result;

  // attempt to load the grasp from the database
  feedback.message = "Requesting grasp demonstration from database...";
  as_.publishFeedback(feedback);
  // TODO load the thing
  graspdb::GraspDemonstration gd;
  if (false)
  {
    result.success = false;
    as_.setSucceeded(result, "Could not load grasp from database.");
    return;
  } else
  {
    // store inside of the result
    result.grasp = gd.toROSPGraspDemonstrationMessage();

    // publish the data
    feedback.message = "Publishing to latched topics...";
    as_.publishFeedback(feedback);
    // send the resulting point cloud message from the goal
    point_cloud_pub_.publish(result.grasp.point_cloud);
    // create and send a PoseStamped
    pose_pub_.publish(gd.getGraspPose().toROSPoseStampedMessage());

    // success
    feedback.message = "Sucecss!";
    as_.publishFeedback(feedback);
    result.success = true;
    as_.setSucceeded(result, "Sucecss!");
  }
}
