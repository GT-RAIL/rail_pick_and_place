#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <carl_moveit/MoveToPoseAction.h>
#include <carl_moveit/CallIK.h>
#include <carl_moveit/CartesianPath.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/LiftAction.h>
#include <rail_grasping/RequestGrasp.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <wpi_jaco_msgs/CartesianCommand.h>

#define NUM_JACO_JOINTS 6

class graspObject
{
public:
  //ROS publishers, subscribers, and action servers
  ros::NodeHandle n;

  ros::Publisher cartesianCommandPub;

  ros::Subscriber armJointSubscriber;

  ros::ServiceClient IKClient;
  ros::ServiceClient cartesianPositionClient;
  ros::ServiceClient cartesianPathClient;

  ros::ServiceServer requestGraspServer;
  ros::ServiceServer requestReleaseServer;

  //action clients
  actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction> acLift;
  actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction> acGripper;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> acJointTrajectory;
  actionlib::SimpleActionClient<carl_moveit::MoveToPoseAction> acMoveArm;

  //tf
  tf::TransformBroadcaster tfBroadcaster;
  tf::TransformListener tfListener;
  tf::Transform graspTransform;

  std::vector<double> armJointPos;
  std::vector<std::string> armJointNames;
  bool jointNamesSet;

  /**
  * Constructor
  */
  graspObject();

  void armJointStatesCallback(const sensor_msgs::JointState &msg);

  bool requestGrasp(rail_grasping::RequestGrasp::Request &req, rail_grasping::RequestGrasp::Response &res);

  bool requestRelease(rail_grasping::RequestGrasp::Request &req, rail_grasping::RequestGrasp::Response &res);

  bool executeGrasp(bool *earlyFailureFlag);

  void publishGraspFrame();
};
