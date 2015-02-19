#include <rail_grasp_collection/GraspCollector.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using namespace std;
using namespace rail::pick_and_place;

GraspCollector::GraspCollector()
    : private_node_("~"),
      ac_wait_time_(AC_WAIT_TIME),
      tf_cache_time_(TF_CACHE_TIME),
      tf_buffer_(tf_cache_time_),
      tf_listener_(tf_buffer_),
      robot_fixed_frame_("base_footprint"),
      grasp_frame_("grasp_link"),
      gripper_action_server_("/manipulation/gripper"),
      lift_action_server_("/manipulation/lift"),
      verify_grasp_action_server_("/manipulation/verify_grasp"),
      as_(private_node_, "store_grasp", boost::bind(&GraspCollector::storeGrasp, this, _1), false)
{
  // set defaults
  debug_ = DEFAULT_DEBUG;

  // grab any parameters we need
  private_node_.getParam("debug", debug_);
  private_node_.getParam("robot_fixed_frame", robot_fixed_frame_);
  private_node_.getParam("grasp_frame", grasp_frame_);
  private_node_.getParam("gripper_action_server", gripper_action_server_);
  private_node_.getParam("lift_action_server", lift_action_server_);
  private_node_.getParam("verify_grasp_action_server", verify_grasp_action_server_);
  private_node_.getParam("finger_frames", finger_frames_);

  // setup a debug publisher if we need it
  if (debug_)
  {
    debug_pub_ = private_node_.advertise<sensor_msgs::PointCloud2>("debug", 1);
  }

  // subscribe to the list of segmented objects
  segmented_objects_sub_ = node_.subscribe("/rail_segmentation/segmented_objects", 1, &GraspCollector::segmentedObjectsCallback, this);

  // setup action clients
  gripper_ac_ = new actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction>(gripper_action_server_, true);
  lift_ac_ = new actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction>(lift_action_server_, true);
  verify_grasp_ac_ = new actionlib::SimpleActionClient<rail_manipulation_msgs::VerifyGraspAction>(verify_grasp_action_server_, true);

  // start the action server
  as_.start();

  ROS_INFO("Grasp Collector Successfully Initialized");
}

GraspCollector::~GraspCollector()
{
  // cleanup actionlib
  as_.shutdown();
  delete gripper_ac_;
  delete lift_ac_;
  delete verify_grasp_ac_;
}

void GraspCollector::storeGrasp(const rail_pick_and_place_msgs::StoreGraspGoalConstPtr &goal)
{
  ROS_INFO("Store grasp requset received.");

  rail_pick_and_place_msgs::StoreGraspFeedback feedback;
  rail_pick_and_place_msgs::StoreGraspResult result;
  // default to false
  result.success = false;

  // request a grasp from the arm
  feedback.message = "Requesting a close gripper action...";
  as_.publishFeedback(feedback);
  rail_manipulation_msgs::GripperGoal gripper_goal;
  gripper_goal.close = true;
  gripper_ac_->sendGoal(gripper_goal);
  if (!gripper_ac_->waitForResult(ac_wait_time_)
      || gripper_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED
      || !gripper_ac_->getResult()->success)
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
    grasp = tf_buffer_.lookupTransform(robot_fixed_frame_, grasp_frame_, ros::Time(0), tf_cache_time_);
  } catch (tf2::TransformException &ex)
  {
    as_.setSucceeded(result, "Could not transform from the grasp frame to the robot fixed frame.");
    return;
  }

  // determine the finger positions
  vector<geometry_msgs::TransformStamped> finger_positions;
  for (size_t i = 0; i < finger_frames_.size(); i++)
  {
    try
    {
      finger_positions.push_back(tf_buffer_.lookupTransform(robot_fixed_frame_, finger_frames_[i], ros::Time(0), tf_cache_time_));
    } catch (tf2::TransformException &ex)
    {
      as_.setSucceeded(result, "Could not transform from the grasp frame to the robot fixed frame.");
      return;
    }
  }

  // check if we are doing a lift
  if (goal->lift)
  {
    // request a lift from the arm
    feedback.message = "Requesting lift...";
    as_.publishFeedback(feedback);
    rail_manipulation_msgs::LiftGoal lift_goal;
    lift_ac_->sendGoal(lift_goal);
    if (!lift_ac_->waitForResult(ac_wait_time_)
        || lift_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED
        || !lift_ac_->getResult()->success)
    {
      as_.setSucceeded(result, "Could not execute lift.");
      return;
    }
  }

  // check if we are doing a grasp verification check
  if (goal->verify)
  {
    // request a lift from the arm
    feedback.message = "Requesting grasp verification...";
    as_.publishFeedback(feedback);
    rail_manipulation_msgs::VerifyGraspGoal verify_grasp_goal;
    verify_grasp_ac_->sendGoal(verify_grasp_goal);
    if (!verify_grasp_ac_->waitForResult(ac_wait_time_)
        || verify_grasp_ac_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED
        || !verify_grasp_ac_->getResult()->success
        || !verify_grasp_ac_->getResult()->grasping)
    {
      as_.setSucceeded(result, "Could not execute lift.");
      return;
    }
  }

  // check for the closest object
  feedback.message = "Searching for the closest segmented object...";
  as_.publishFeedback(feedback);
  int closest = this->determineClosestObject(grasp.transform.translation);
  if (closest < 0)
  {
    as_.setSucceeded(result, "No segmented objects found.");
    return;
  }
  // check if we need to transform the point cloud
  rail_manipulation_msgs::SegmentedObject &object = object_list_.objects[closest];
  if (object.cloud.header.frame_id != robot_fixed_frame_)
  {
    try
    {
      sensor_msgs::PointCloud2 transformed_cloud = tf_buffer_.transform(object.cloud, robot_fixed_frame_, tf_cache_time_);
      object.cloud = transformed_cloud;
    } catch (tf2::TransformException &ex)
    {
      as_.setSucceeded(result, "Could not transform the segemented object to the robot fixed frame.");
      return;
    }
  }
  // check if we are going to publish some debug info
  if (debug_)
  {
    debug_pub_.publish(object.cloud);
  }

  // store the data
  feedback.message = "Storing grasp data...";
  as_.publishFeedback(feedback);
  // TODO: store the data

  // success
  result.success = true;
  as_.setSucceeded(result, "Sucecss!");
}

void GraspCollector::segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &object_list)
{
  ROS_INFO("Updated segmented object list received.");
  object_list_ = object_list;
}

int GraspCollector::determineClosestObject(const geometry_msgs::Vector3 &v)
{
  // check if we actually have some objects
  if (object_list_.objects.size() == 0)
  {
    return -1;
  } else if (object_list_.objects.size() == 1)
  {
    // no need to check if we only have a single object
    return 0;
  } else
  {
    // find the closest point
    float min = numeric_limits<float>::infinity();
    int index = 0;
    // check each segmented object
    for (size_t i = 0; i < object_list_.objects.size(); i++)
    {
      //convert PointCloud2 to PointCloud to access the data easily
      sensor_msgs::PointCloud cloud;
      sensor_msgs::convertPointCloud2ToPointCloud(object_list_.objects[i].cloud, cloud);
      // check each point in the cloud
      for (unsigned int j = 0; j < cloud.points.size(); j++)
      {
        // euclidean distance to the point
        float dist = sqrt(pow(cloud.points[j].x - v.x, 2) + pow(cloud.points[j].y - v.y, 2) + pow(cloud.points[j].z - v.z, 2));
        if (dist < min)
        {
          min = dist;
          index = i;
        }
      }
    }
    return index;
  }
}