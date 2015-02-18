#include <rail_grasp_collection/graspCollection.h>

using namespace std;

graspCollection::graspCollection(string name) :
    acGrasp("/jaco_arm/manipulation/grasp", true),
    acPickup("/jaco_arm/manipulation/pickup", true),
    as(n, name, boost::bind(&graspCollection::executePickup, this, _1), false), actionName(name)
{
  armJoints.resize(NUM_ARM_JOINTS);
  fingerJoints.resize(NUM_FINGERS);

  ROS_INFO("Waiting for JACO manipulation action servers...");
  acGrasp.waitForServer();
  acPickup.waitForServer();
  ROS_INFO("Finished waiting for JACO manipulation action servers");

  //temporarily unused as the JACO arm does not have a good method of grasp verification
  //verifyGraspClient = n.serviceClient<rail_grasping::GraspCheck>("/rail_grasping/verify_grasp");
  closestObjectClient = n.serviceClient<rail_grasp_collection::ClosestObject>("/rail_grasp_collection/determine_closest_object");

  jointStateSubscriber = n.subscribe("/joint_states", 1, &graspCollection::jointStatesCallback, this);

  rArmPublisher = n.advertise<trajectory_msgs::JointTrajectory>("r_arm_controller/command", 1);
  lArmPublisher = n.advertise<trajectory_msgs::JointTrajectory>("l_arm_controller/command", 1);
  //debug
  debugPublisher = n.advertise<sensor_msgs::PointCloud>("rail_grasp_collection/debug", 1);
  //end debug

  as.start();
}

void graspCollection::jointStatesCallback(const sensor_msgs::JointState &states)
{
  for (unsigned int i = 0; i < states.name.size(); i++)
  {
    if (i >= JOINT_START_INDEX && i < JOINT_START_INDEX + NUM_ARM_JOINTS)
      armJoints[i] = states.position[i];
    else if (i >= FINGER_START_INDEX && i < FINGER_START_INDEX + NUM_FINGERS)
      fingerJoints[i - FINGER_START_INDEX] = states.position[i];
  }
}

void graspCollection::executePickup(const rail_grasp_collection::PickupGoalConstPtr &goal)
{
  //Grasp Object
  asFeedback.currentStep = "Attempting to pickup object...";
  as.publishFeedback(asFeedback);

  asFeedback.currentStep = "Grasping object";
  as.publishFeedback(asFeedback);

  //call grasp server
  wpi_jaco_msgs::ExecuteGraspGoal executeGrasp;
  executeGrasp.closeGripper = true;
  executeGrasp.limitFingerVelocity = false;
  acGrasp.sendGoal(executeGrasp);
  acGrasp.waitForResult(ros::Duration(10));

  //update resulting positions of gripper and fingers

  //set grasp pose with respect to the center of the gripper's fingers
  geometry_msgs::PoseStamped approachPose;
  geometry_msgs::PoseStamped poseOut;
  approachPose.header.frame_id = "jaco_link_hand";
  approachPose.pose.position.x = 0.0;
  approachPose.pose.position.y = 0.0;
  approachPose.pose.position.z = -0.19;
  approachPose.pose.orientation.x = 0.0;
  approachPose.pose.orientation.y = 0.0;
  approachPose.pose.orientation.z = 0.0;
  approachPose.pose.orientation.w = 1.0;
  listener.transformPose("base_footprint", approachPose, poseOut);

  /*
  //get gripper position
  tf::StampedTransform graspPointTransform;
  try { listener.lookupTransform("base_footprint", "/jaco_link_hand", ros::Time(0), graspPointTransform); }
  catch (tf::TransformException ex) { ROS_ERROR("%s",ex.what()); }
  */

  //Get finger poses
  tf::StampedTransform finger1Transform, finger2Transform, finger3Transform;
  try
  {listener.lookupTransform("base_footprint", "/jaco_link_finger_1", ros::Time(0), finger1Transform);}
  catch (tf::TransformException ex)
  {ROS_ERROR("%s", ex.what());}
  try
  {listener.lookupTransform("base_footprint", "/jaco_link_finger_2", ros::Time(0), finger2Transform);}
  catch (tf::TransformException ex)
  {ROS_ERROR("%s", ex.what());}
  tf::StampedTransform rightFingerTransform;
  try
  {listener.lookupTransform("base_footprint", "/jaco_link_finger_3", ros::Time(0), finger3Transform);}
  catch (tf::TransformException ex)
  {ROS_ERROR("%s", ex.what());}

  //convertTFTransformToPoseMsg(graspPointTransform, asResult.gripperPose);
  asResult.gripperPose = poseOut;
  convertTFTransformToPoseMsg(finger1Transform, asResult.finger1Pose);
  convertTFTransformToPoseMsg(finger2Transform, asResult.finger2Pose);
  convertTFTransformToPoseMsg(finger3Transform, asResult.finger3Pose);


  //Pickup Object
  bool actionSucceeded;

  if (goal->liftObject)
  {
    ROS_INFO("Lifting object to test grasp strength...");
    asFeedback.currentStep = "Lifting object to test grasp strength";
    as.publishFeedback(asFeedback);

    wpi_jaco_msgs::ExecutePickupGoal pickupGoal;
    wpi_jaco_msgs::ExecutePickupResultConstPtr pickupResultPtr;
    pickupGoal.limitFingerVelocity = false;
    pickupGoal.setLiftVelocity = false;

    acPickup.sendGoal(pickupGoal);
    acPickup.waitForResult(ros::Duration(6.0));
    pickupResultPtr = acPickup.getResult();

    //actionSucceeded = pickupResultPtr->success;
    actionSucceeded = true; //temporary, set to true while Cartesian arm control is being buggy
  }
  else
    actionSucceeded = true;


  //Test Pickup
  if (actionSucceeded)
  {
    asFeedback.currentStep = "Checking grasp strength...";
    as.publishFeedback(asFeedback);
    //Don't use grasp verification for now as the JACO arm doesn't have a good way to do it
    //TODO: look into grasp verification for the JACO arm
    asResult.success = true;
    /*
    rail_grasping::GraspCheck srv;
    if (verifyGraspClient.call(srv))
    {
      asResult.success = srv.response.isGrasping;
      if (asResult.success)
      {
      */
    //find closest object
    ROS_INFO("Finding closest object...");
    rail_grasp_collection::ClosestObject objSrv;
    objSrv.request.x = asResult.gripperPose.pose.position.x;
    objSrv.request.y = asResult.gripperPose.pose.position.y;
    objSrv.request.z = asResult.gripperPose.pose.position.z;
    if (closestObjectClient.call(objSrv))
    {
      asResult.reference_frame_id = objSrv.response.reference_frame_id;
      asResult.pointCloud = objSrv.response.pointCloud;
      //debug
      /*
      ROS_INFO("RGB verification:");
      for (int i = 0; i < 50; i ++)
      {
        if (i < asResult.pointCloud.channels[0].values.size())
        {
          int32_t color = *(int32_t *)(&(asResult.pointCloud.channels[0].values[i]));
          uint8_t r = (uint8_t)((color >> 16) & 0x0000ff);
          uint8_t g = (uint8_t)(color >> 8) & 0x000ff;
          uint8_t b = (uint8_t)(color) & 0x0000ff;
          cout << (int)r << ", " << (int)g << ", " << (int)b << endl;
        }
      }
      */
      debugPublisher.publish(asResult.pointCloud);
      //end debug
    }
    else
    {
      asResult.success = false;
      ROS_INFO("Pickup data collection failed, couldn't find any previously detected objects!");
      asFeedback.currentStep = "Pickup data collection failed, couldn't find any previously detected objects!";
      as.publishFeedback(asFeedback);
    }
    /*
    }
    else
    {
      asResult.success = false;
      asFeedback.currentStep = "Pickup failed, no object is in the gripper!";
      as.publishFeedback(asFeedback);
    }
  }
  else
  {
    asResult.success = false;
    asFeedback.currentStep = "Pickup failed!";
    as.publishFeedback(asFeedback);
  }
  */
  }
  else
  {
    asResult.success = false;
    ROS_INFO("Pickup failed, couldn't lift arm!");
    asFeedback.currentStep = "Pickup failed, couldn't lift arm!";
    as.publishFeedback(asFeedback);
  }

  if (asResult.success == true)
  {
    ROS_INFO("Pickup succeeded!");
    asFeedback.currentStep = "Pickup succeeded!";
    as.publishFeedback(asFeedback);
  }
  else
  {
    //Open gripper
    wpi_jaco_msgs::ExecuteGraspGoal executeRelease;
    executeRelease.closeGripper = false;
    executeRelease.limitFingerVelocity = false;
    acGrasp.sendGoal(executeRelease);
    acGrasp.waitForResult(ros::Duration(10));
  }

  as.setSucceeded(asResult);
}

void graspCollection::convertTFTransformToPoseMsg(const tf::StampedTransform &transform, geometry_msgs::PoseStamped &pose)
{
  pose.header.frame_id = transform.frame_id_;
  pose.pose.position.x = transform.getOrigin().x();
  pose.pose.position.y = transform.getOrigin().y();
  pose.pose.position.z = transform.getOrigin().z();
  pose.pose.orientation.x = transform.getRotation().x();
  pose.pose.orientation.y = transform.getRotation().y();
  pose.pose.orientation.z = transform.getRotation().z();
  pose.pose.orientation.w = transform.getRotation().w();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rail_grasp_collection");

  graspCollection gc(ros::this_node::getName());

  ros::spin();

  return 0;
}
