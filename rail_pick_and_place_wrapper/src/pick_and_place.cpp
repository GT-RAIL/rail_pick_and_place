#include "rail_pick_and_place_wrapper/pick_and_place.h"

using namespace std;

pickAndPlace::pickAndPlace() : acHome("jaco_arm/home_arm", true)
{
  objectSubscriber = n.subscribe("rail_segmentation/segmented_objects", 1, &pickAndPlace::objectCallback, this);

  pickupServer = n.advertiseService("rail_pick_and_place/pickup_object", &pickAndPlace::pickup, this);
  pickupSegmentedServer = n.advertiseService("rail_pick_and_place/pickup_segmented_object", &pickAndPlace::pickupSegmented, this);

  recognizeAndGraspClient = n.serviceClient<rail_pick_and_place_msgs::RecognizeAndGrasp>("rail_recognition/recognize_and_pickup");
  graspRecognizedClient = n.serviceClient<rail_pick_and_place_msgs::GraspRecognized>("rail_recognition/grasp_recognized");
  segmentClient = n.serviceClient<rail_segmentation::Segment>("rail_segmentation/segment");
  
  ROS_INFO("Waiting for arm action server...");
  acHome.waitForServer();
  ROS_INFO("Finished waiting for arm action server.");
}

bool pickAndPlace::pickup(rail_pick_and_place_msgs::PickupObject::Request &req, rail_pick_and_place_msgs::PickupObject::Response &res)
{
  rail_pick_and_place_msgs::RecognizeAndGrasp::Request pickupReq;
  rail_pick_and_place_msgs::RecognizeAndGrasp::Response pickupRes;
  
  //set object models to be picked up
  pickupReq.objectIndices.clear();
  if (req.objectName.compare("bowl") == 0)
  {
    pickupReq.objectIndices.push_back(0);
  }
  else if (req.objectName.compare("cup") == 0)
  {
    pickupReq.objectIndices.push_back(1);
  }
  else if (req.objectName.compare("fork") == 0 || req.objectName.compare("spoon") == 0)
  {
    pickupReq.objectIndices.push_back(2);
  }

  int attempts = 0;
  
  while (attempts < 1)  //Adjust for automatic retries on failure
  {
    //retract arm
    ROS_INFO("Retracting arm...");
    wpi_jaco_msgs::HomeArmGoal retractGoal;
    retractGoal.retract = true;
    retractGoal.retractPosition.position = true;
    retractGoal.retractPosition.armCommand = true;
    retractGoal.retractPosition.fingerCommand = false;
    retractGoal.retractPosition.repeat = false;
    retractGoal.retractPosition.joints.resize(6);
    retractGoal.retractPosition.joints[0] = -2.57;
    retractGoal.retractPosition.joints[1] = 1.39;
    retractGoal.retractPosition.joints[2] = .527;
    retractGoal.retractPosition.joints[3] = -.084;
    retractGoal.retractPosition.joints[4] = .515;
    retractGoal.retractPosition.joints[5] = -1.745;
    acHome.sendGoal(retractGoal);
    acHome.waitForResult(ros::Duration(15.0));
    ros::Duration(9.0).sleep();
  
    //call segmentation
    ROS_INFO("Segmenting...");
    rail_segmentation::Segment::Request segmentReq;
    rail_segmentation::Segment::Response segmentRes;
    segmentReq.useMapFrame = false;
    segmentReq.clear = true;
    segmentReq.segmentOnRobot = false;
    if (!segmentClient.call(segmentReq, segmentRes))
    {
      ROS_INFO("Error calling segmentation client.");
      res.success = false;
      return false;
    }
    
    //home arm
    ROS_INFO("Readying arm...");
    wpi_jaco_msgs::HomeArmGoal homeGoal;
    homeGoal.retract = false;
    acHome.sendGoal(homeGoal);
    acHome.waitForResult(ros::Duration(15.0));
    
    //attempt to pickup the requested object
    for (unsigned int i = 0; i < segmentRes.objects.size(); i ++)
    {
      ROS_INFO("Starting pickup...");
      sensor_msgs::convertPointCloud2ToPointCloud(segmentRes.objects[i], pickupReq.cloud);
      pickupReq.numAttempts = 6;
      recognizeAndGraspClient.call(pickupReq, pickupRes);
      if (pickupRes.success)
      {
        ROS_INFO("Object pickup complete.");
        res.success = true;
        return true;
      }
    }
    
    attempts ++;
  }
  
  ROS_INFO("Object pickup failed.");
  res.success = false;
  return true;
}

void pickAndPlace::objectCallback(const rail_segmentation::SegmentedObjectList& list)
{
  ROS_INFO("Received new objects");
  objectList = list;
}

bool pickAndPlace::pickupSegmented(rail_pick_and_place_msgs::PickupSegmentedObject::Request &req,
    rail_pick_and_place_msgs::PickupSegmentedObject::Response &res)
{
  ROS_INFO("Starting pickup...");

  if (req.objectIndex >= objectList.objects.size())
  {
    ROS_INFO("Pickup index out of bounds of segmented object list, pickup failed.");
    res.success = false;
    return true;
  }

  if (objectList.objects[req.objectIndex].recognized)
  {
    //perform previously calculated grasps from recognition
    rail_pick_and_place_msgs::GraspRecognized::Request graspReq;
    rail_pick_and_place_msgs::GraspRecognized::Response graspRes;
    graspReq.numAttempts = 6;
    graspReq.grasps = objectList.objects[req.objectIndex].graspPoses;
    graspReq.objectIndex = objectList.objects[req.objectIndex].model;
    
    if (!graspRecognizedClient.call(graspReq, graspRes))
    {
      ROS_INFO("Grasping service client call failed.");
      res.success = false;
      return false;
    }
    
    if (graspRes.success)
    {
      ROS_INFO("Object pickup complete.");
      res.success = true;
      return true;
    }
    else
    {
      ROS_INFO("Object pickup failed.");
      res.success = false;
      return true;
    }
    
  }
  else
  {
    //recognize and pickup object
    rail_pick_and_place_msgs::RecognizeAndGrasp::Request pickupReq;
    rail_pick_and_place_msgs::RecognizeAndGrasp::Response pickupRes;
    
    pickupReq.objectIndices.clear();
    sensor_msgs::convertPointCloud2ToPointCloud(objectList.objects[req.objectIndex].objectCloud, pickupReq.cloud);
    pickupReq.numAttempts = 6;
    
    recognizeAndGraspClient.call(pickupReq, pickupRes);
    if (pickupRes.success)
    {
      ROS_INFO("Object pickup complete.");
      res.success = true;
    }
    else
    {
      ROS_INFO("Pickup failed.");
      res.success = false;
    }
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rail_pick_and_place");
  
  pickAndPlace p;
  
  ros::spin();
  
  return 0;
}
