#include "rail_grasping/graspObject.h"

using namespace std;

graspObject::graspObject() :
  acPickup("jaco_arm/manipulation/pickup"),
  acGrasp("jaco_arm/manipulation/grasp"),
  acJointTrajectory("jaco_arm/joint_velocity_controller/trajectory"),
  acMoveArm("carl_moveit_wrapper/move_to_pose")
{
  ROS_INFO("Starting rail grasping...");
  graspTransform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  graspTransform.setRotation(tf::Quaternion(0, 0, 0, 1));
  
  armJointSubscriber = n.subscribe("jaco_arm/joint_states", 1, &graspObject::armJointStatesCallback, this);
  
  IKClient = n.serviceClient<carl_moveit::CallIK>("carl_moveit_wrapper/call_ik");
  
  jointNamesSet = false;
  armJointPos.resize(NUM_JACO_JOINTS);
  armJointNames.resize(NUM_JACO_JOINTS);
  
  while ( !acPickup.waitForServer(ros::Duration(5.0)) &&
          !acGrasp.waitForServer(ros::Duration(5.0)) &&
          !acJointTrajectory.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for grasp, pickup, and arm trajectory action servers...");
  }
  
  requestGraspServer = n.advertiseService("rail_grasping/request_grasp", &graspObject::requestGrasp, this);
  
  ROS_INFO("Rail grasping started.");
}

void graspObject::armJointStatesCallback(const sensor_msgs::JointState& msg)
{
  for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
  {
    armJointPos[i] = msg.position[i];
  }
  
  if (!jointNamesSet)
  {
    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i ++)
    {
      armJointNames[i] = msg.name[i];
    }
    jointNamesSet = true;
  }
}

bool graspObject::requestGrasp(rail_grasping::RequestGrasp::Request &req, rail_grasping::RequestGrasp::Response &res)
{
  ROS_INFO("Received new grasp request...");
  //set new grasp position
  graspTransform.setOrigin(tf::Vector3(req.graspPose.position.x, req.graspPose.position.y, req.graspPose.position.z));
  graspTransform.setRotation(tf::Quaternion(req.graspPose.orientation.x, req.graspPose.orientation.y, req.graspPose.orientation.z, req.graspPose.orientation.w));
  tfBroadcaster.sendTransform(tf::StampedTransform(graspTransform, ros::Time::now(), "base_footprint", "grasp_frame"));
  
  ros::Duration(1).sleep();

  bool earlyFailureDetected;
  res.result = executeGrasp(&earlyFailureDetected);
  res.earlyFailureDetected = earlyFailureDetected;

  return true;
}

bool graspObject::executeGrasp(bool *earlyFailureFlag)
{
  ROS_INFO("\nGrasp execution started");
  //*********** Align gripper on approach angle ***********
  //Calculate pose on approach angle
  geometry_msgs::PoseStamped approachPose;
  geometry_msgs::PoseStamped poseOut;
  //approachPose.header.stamp = ros::Time::now();
  approachPose.header.frame_id = "grasp_frame";
  approachPose.pose.position.x = 0.0; //TODO: confirm, was previously -0.1
  approachPose.pose.position.y = 0.0;
  approachPose.pose.position.z = 0.1; //TODO: confirm, was previously 0.0
  approachPose.pose.orientation.x = 0.0;
  approachPose.pose.orientation.y = 0.0;
  approachPose.pose.orientation.z = 0.0;
  approachPose.pose.orientation.w = 1.0;
  
  //adjustment to put approach pose from centered at fingers to jaco_link_hand
  approachPose.pose.position.z += .19; //TODO: test this
  
  tfListener.transformPose("base_footprint", approachPose, poseOut);
  
  //Send goal to move arm service
  ROS_INFO("Moving to approach angle...");
  carl_moveit::MoveToPoseGoal movePoseGoal;
  movePoseGoal.pose = poseOut.pose;
  acMoveArm.sendGoal(movePoseGoal);
  ROS_INFO("Approach angle arm move iniated.");
  acMoveArm.waitForResult(ros::Duration(10.0));
  carl_moveit::MoveToPoseResultConstPtr movePoseResult = acMoveArm.getResult();
  
  *earlyFailureFlag = !movePoseResult->success;
  if (*earlyFailureFlag)
  {
    ROS_INFO("Detected failure on moving to approach angle.");
    return false;
  }
    
  //wait for asynchronous trajectory execution to complete
  //TODO: Update this to listen to the trajectory execution action server state for completion
  ROS_INFO("Waiting for trajectory execution...");
  ros::Duration(10.0).sleep();
  ROS_INFO("Done waiting for trajectory execution.");
  
  //********** Move gripper along approach angle ***********
  //Open gripper
  ROS_INFO("Fully opening gripper...");
  wpi_jaco_msgs::ExecuteGraspGoal openGripperGoal;
  openGripperGoal.closeGripper = false;
  openGripperGoal.limitFingerVelocity = false;
  acGrasp.sendGoal(openGripperGoal);
  acGrasp.waitForResult(ros::Duration(5.0));
  ROS_INFO("Gripper opened.");
  
  //Determine pickup pose
  geometry_msgs::PoseStamped pickupPose;
  geometry_msgs::PoseStamped pickupPoseOut;
  pickupPoseOut.header.frame_id = "base_footprint";
  //pickupPose.header.stamp = ros::Time::now();
  pickupPose.header.frame_id = "grasp_frame";
  pickupPose.pose.position.x = 0.0;
  pickupPose.pose.position.y = 0.0;
  pickupPose.pose.position.z = 0.0;
  pickupPose.pose.orientation.x = 0.0;
  pickupPose.pose.orientation.y = 0.0;
  pickupPose.pose.orientation.z = 0.0;
  pickupPose.pose.orientation.w = 1.0;
  
  //adjustment to put approach pose from centered at fingers to jaco_link_hand
  approachPose.pose.position.z += .19; //TODO: test this
  
  tfListener.transformPose("base_footprint", pickupPose, pickupPoseOut);
  
  ROS_INFO("Moving arm to pickup pose...");
  /************* Arm Movement With IK *****************/
  if (!moveArmToPose(pickupPoseOut))
  {
    ROS_INFO("Failed to move arm to pickup pose");
    *earlyFailureFlag = true;
    return false;
  }
  ROS_INFO("Arm at pickup pose.");
  
  //Close gripper
  ROS_INFO("Closing gripper...");
  wpi_jaco_msgs::ExecuteGraspGoal closeGripperGoal;
  closeGripperGoal.closeGripper = true;
  closeGripperGoal.limitFingerVelocity = false;
  acGrasp.sendGoal(closeGripperGoal);
  acGrasp.waitForResult(ros::Duration(5.0));
  ROS_INFO("Gripper closed.");
  
  //*********** lift object ***********
  /* Lift object using pickup action server instead, switch it out with this if the
   * Cartesian lift is not working consistantly
   */
  ROS_INFO("Lifting object...");
  wpi_jaco_msgs::ExecutePickupGoal pickupGoal;
  pickupGoal.limitFingerVelocity = false;
  pickupGoal.setLiftVelocity = false;
  acPickup.sendGoal(pickupGoal);
  acPickup.waitForResult(ros::Duration(10));
  ROS_INFO("Object lifting complete.");
  
  /*
  //Calculate lift pose
  geometry_msgs::PoseStamped liftPose;
  geometry_msgs::PoseStamped liftPoseOut;
  liftPoseOut.header.frame_id = "base_footprint";
  //liftPose.header.stamp = ros::Time::now();
  if (side.compare("left") == 0)
    liftPose.header.frame_id = "l_wrist_roll_link";
  else
    liftPose.header.frame_id = "r_wrist_roll_link";
  liftPose.pose.position.x = 0.0;
  liftPose.pose.position.y = 0.0;
  liftPose.pose.position.z = 0.0;
  liftPose.pose.orientation.x = 0.0;
  liftPose.pose.orientation.y = 0.0;
  liftPose.pose.orientation.z = 0.0;
  liftPose.pose.orientation.w = 1.0;
  
  tfListener.transformPose("base_footprint", liftPose, liftPoseOut);
  
  liftPoseOut.pose.position.z += 0.1;
  
  if (!moveArmToPose(side, liftPoseOut))
  {
    ROS_INFO("Failed to move arm to lift pose");
    return false;
  }
  */
  
  return true;
}

bool graspObject::moveArmToPose(geometry_msgs::PoseStamped poseGoal)
{
  //IK
  carl_moveit::CallIK::Request ikReq;
  carl_moveit::CallIK::Response ikRes;
  ikReq.pose = poseGoal.pose;
  IKClient.call(ikReq, ikRes);
  
  if (ikRes.success = false)
  {
    ROS_INFO("IK Server returned false");
    return false;
  }
  
  //Create and send trajectory to the arm
  //possibly send to arm directly as a joint goal with speed limitation?
  if (!jointNamesSet)
  {
    ROS_INFO("Joint data from the arm has not been received");
    return false;
  }
  
  trajectory_msgs::JointTrajectoryPoint startPoint;
  trajectory_msgs::JointTrajectoryPoint endPoint;
  startPoint.positions = armJointPos;
  endPoint.positions = ikRes.jointPositions;
  control_msgs::FollowJointTrajectoryGoal trajGoal;
  trajGoal.trajectory.joint_names = armJointNames;
  trajGoal.trajectory.points.clear();
  trajGoal.trajectory.points.push_back(startPoint);
  trajGoal.trajectory.points.push_back(endPoint);
  //trajGoal.goal_tolerance.clear();
  //TODO: determine if setting joint goal tolerances is required
  //trajGoal.goal_time_tolerance = ros::Duration(10.0);  //TODO: This is adjustable
  
  ROS_INFO("Sending trajectory goal directly to arm");
  
  acJointTrajectory.sendGoal(trajGoal);
  acJointTrajectory.waitForResult(ros::Duration(10.0));
  
  //TODO: check result of trajectory execution to determine return value
  return true;
}

void graspObject::publishGraspFrame()
{
  tfBroadcaster.sendTransform(tf::StampedTransform(graspTransform, ros::Time::now(), "base_footprint", "grasp_frame"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rail_grasping");
  
  graspObject go;
  
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    go.publishGraspFrame();
    loop_rate.sleep();
  }
  
  return 0;
}
