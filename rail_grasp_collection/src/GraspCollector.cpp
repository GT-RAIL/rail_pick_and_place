/*!
 * \file GraspCollector.cpp
 * \brief The main grasp collector node object.
 *
 * The grasp collector is responsible for capturing and storing grasps. An action server is started is the main
 * entry point to grasp collecting.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 3, 2015
 */

// RAIL Grasp Collection
#include "rail_grasp_collection/GraspCollector.h"

// ROS
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using namespace std;
using namespace rail::pick_and_place;

GraspCollector::GraspCollector()
    : private_node_("~"), ac_wait_time_(AC_WAIT_TIME), tf_listener_(tf_buffer_),
      robot_fixed_frame_id_("base_footprint"), eef_frame_id_("eef_link"),
      as_(private_node_, "grasp_and_store", boost::bind(&GraspCollector::graspAndStore, this, _1), false)
{
  // set defaults
  debug_ = DEFAULT_DEBUG;
  int port = graspdb::Client::DEFAULT_PORT;
  string segmented_objects_topic("/segmentation/segmented_objects");
  string gripper_action_server("/manipulation/gripper");
  string lift_action_server("/manipulation/lift");
  string verify_grasp_action_server("/manipulation/verify_grasp");
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");

  // grab any parameters we need
  private_node_.getParam("debug", debug_);
  private_node_.getParam("robot_fixed_frame_id", robot_fixed_frame_id_);
  private_node_.getParam("eef_frame_id", eef_frame_id_);
  private_node_.getParam("segmented_objects_topic", segmented_objects_topic);
  private_node_.getParam("gripper_action_server", gripper_action_server);
  private_node_.getParam("lift_action_server", lift_action_server);
  private_node_.getParam("verify_grasp_action_server", verify_grasp_action_server);
  node_.getParam("/graspdb/host", host);
  node_.getParam("/graspdb/port", port);
  node_.getParam("/graspdb/user", user);
  node_.getParam("/graspdb/password", password);
  node_.getParam("/graspdb/db", db);

  // set up a connection to the grasp database
  graspdb_ = new graspdb::Client(host, port, user, password, db);
  okay_ = graspdb_->connect();

  // setup a debug publisher if we need it
  if (debug_)
  {
    debug_pub_ = private_node_.advertise<sensor_msgs::PointCloud2>("debug", 1, true);
  }

  // subscribe to the list of segmented objects
  segmented_objects_sub_ = node_.subscribe(segmented_objects_topic, 1, &GraspCollector::segmentedObjectsCallback,
                                           this);

  // setup action clients
  gripper_ac_ = new actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction>(gripper_action_server, true);
  lift_ac_ = new actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction>(lift_action_server, true);
  verify_grasp_ac_ = new actionlib::SimpleActionClient<rail_manipulation_msgs::VerifyGraspAction>(
      verify_grasp_action_server, true
  );

  // start the action server
  as_.start();

  if (okay_)
  {
    ROS_INFO("Grasp Collector Successfully Initialized");
  }
}

GraspCollector::~GraspCollector()
{
  // cleanup
  as_.shutdown();
  graspdb_->disconnect();
  delete gripper_ac_;
  delete lift_ac_;
  delete verify_grasp_ac_;
  delete graspdb_;
}

bool GraspCollector::okay() const
{
  return okay_;
}

void GraspCollector::graspAndStore(const rail_pick_and_place_msgs::GraspAndStoreGoalConstPtr &goal)
{
  ROS_INFO("Store grasp requset received.");

  rail_pick_and_place_msgs::GraspAndStoreFeedback feedback;
  rail_pick_and_place_msgs::GraspAndStoreResult result;
  // default to false
  result.success = false;
  result.id = 0;

  // used for action server checks
  bool completed, succeeded, success;

  // request a grasp from the arm
  feedback.message = "Requesting a close gripper action...";
  as_.publishFeedback(feedback);
  rail_manipulation_msgs::GripperGoal gripper_goal;
  gripper_goal.close = true;
  gripper_ac_->sendGoal(gripper_goal);
  completed = gripper_ac_->waitForResult(ac_wait_time_);
  succeeded = (gripper_ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  success = gripper_ac_->getResult()->success;
  if (!completed || !succeeded || !success)
  {
    as_.setSucceeded(result, "Could not close the gripper.");
    return;
  }

  // get the grasp position information
  feedback.message = "Determinging grasp position...";
  as_.publishFeedback(feedback);
  // get the TF from the buffer
  geometry_msgs::TransformStamped grasp;
  try
  {
    grasp = tf_buffer_.lookupTransform(robot_fixed_frame_id_, eef_frame_id_, ros::Time(0));
  } catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    as_.setSucceeded(result, "Could not transform from the grasp frame to the robot fixed frame.");
    return;
  }

  // check if we are doing a lift
  if (goal->lift)
  {
    // request a lift from the arm
    feedback.message = "Requesting lift...";
    as_.publishFeedback(feedback);
    rail_manipulation_msgs::LiftGoal lift_goal;
    lift_ac_->sendGoal(lift_goal);
    completed = lift_ac_->waitForResult(ac_wait_time_);
    succeeded = (lift_ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
    success = lift_ac_->getResult()->success;
    if (!completed || !succeeded || !success)
    {
      as_.setSucceeded(result, "Could not execute lift.");
      return;
    }
  }

  // check if we are doing a grasp verification check
  if (goal->verify)
  {
    // request a grasp verification from the arm
    feedback.message = "Requesting grasp verification...";
    as_.publishFeedback(feedback);
    rail_manipulation_msgs::VerifyGraspGoal verify_grasp_goal;
    verify_grasp_ac_->sendGoal(verify_grasp_goal);
    completed = verify_grasp_ac_->waitForResult(ac_wait_time_);
    succeeded = (verify_grasp_ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
    rail_manipulation_msgs::VerifyGraspResultConstPtr verify_result = verify_grasp_ac_->getResult();
    success = verify_result->success;
    if (!completed || !succeeded || !success)
    {
      as_.setSucceeded(result, "Could not verify grasp.");
      return;
    } else if (!verify_result->grasping)
    {
      as_.setSucceeded(result, "Grasp is not verified.");
      return;
    }
  }

  // check for the closest object
  feedback.message = "Searching for the closest segmented object...";
  as_.publishFeedback(feedback);
  // lock for the vector
  {
    boost::mutex::scoped_lock lock(mutex_);
    // check if we actually have some objects
    int closest = 0;
    if (object_list_->objects.size() == 0)
    {
      as_.setSucceeded(result, "No segmented objects found.");
      return;
    } else if (object_list_->objects.size() > 1)
    {
      // find the closest point
      float min = numeric_limits<float>::infinity();
      //geometry_msgs::Vector3 &v = grasp.transform.translation;
      // check each segmented object
      for (size_t i = 0; i < object_list_->objects.size(); i++)
      {
        geometry_msgs::TransformStamped eef_transform = tf_buffer_.lookupTransform(
            object_list_->objects[i].point_cloud.header.frame_id, eef_frame_id_, ros::Time(0)
        );
        geometry_msgs::Vector3 &v = eef_transform.transform.translation;
        //convert PointCloud2 to PointCloud to access the data easily
        sensor_msgs::PointCloud cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(object_list_->objects[i].point_cloud, cloud);
        // check each point in the cloud
        for (size_t j = 0; j < cloud.points.size(); j++)
        {
          // euclidean distance to the point
          float dist = sqrt(
              pow(cloud.points[j].x - v.x, 2) + pow(cloud.points[j].y - v.y, 2) + pow(cloud.points[j].z - v.z, 2)
          );
          if (dist < min)
          {
            min = dist;
            closest = i;
          }
        }
      }
    }
    // check if we need to transform the point cloud
    rail_manipulation_msgs::SegmentedObject &object = object_list_->objects[closest];
    if (object.point_cloud.header.frame_id != robot_fixed_frame_id_)
    {
      try
      {
        sensor_msgs::PointCloud2 transformed_cloud = tf_buffer_.transform(object.point_cloud, robot_fixed_frame_id_,
                                                                          ros::Time(0),
                                                                          object.point_cloud.header.frame_id);
        object.point_cloud = transformed_cloud;
        object.point_cloud.header.frame_id = robot_fixed_frame_id_;
      } catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        as_.setSucceeded(result, "Could not transform the segemented object to the robot fixed frame.");
        return;
      }
    }
    // check if we are going to publish some debug info
    if (debug_)
    {
      debug_pub_.publish(object.point_cloud);
    }

    // store the data
    feedback.message = "Storing grasp data...";
    as_.publishFeedback(feedback);
    graspdb::GraspDemonstration gd(goal->object_name, graspdb::Pose(grasp), eef_frame_id_, object.point_cloud,
                                   object.image);
    if (graspdb_->addGraspDemonstration(gd))
    {
      // store the ID
      result.id = gd.getID();
    } else
    {
      as_.setSucceeded(result, "Could not insert into database.");
      return;
    }
  }

  // success
  result.success = true;
  as_.setSucceeded(result, "Success!");
}

void GraspCollector::segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList::Ptr &object_list)
{
  ROS_INFO("Updated segmented object list received.");
  // lock for the vector
  boost::mutex::scoped_lock lock(mutex_);
  object_list_ = object_list;
}
