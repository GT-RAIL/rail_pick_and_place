#include "rail_recognition/pcRecognition.h"

using namespace std;
using namespace pcl;

pcRecognition::pcRecognition()
{
  xTrans = 0.0;
  yTrans = 0.0;
  zTrans = 0.0;

  training = false;

  releasePosePublisher = n.advertise<geometry_msgs::PoseStamped>("rail_recognition/release_pose", 1);
  readGraspClient = n.serviceClient<rail_recognition::ReadGrasp>("grasp_reader/read_grasps");
  requestGraspClient = n.serviceClient<rail_grasping::RequestGrasp>("rail_grasping/request_grasp");
  requestReleaseClient = n.serviceClient<rail_grasping::RequestGrasp>("rail_grasping/request_release");
  recognizeServer = n.advertiseService("rail_recognition/recognize", &pcRecognition::recognize, this);
  recognizeAndGraspServer = n.advertiseService("rail_recognition/recognize_and_pickup", &pcRecognition::recognizeAndPickup, this);
  graspRecognizedServer = n.advertiseService("rail_recognition/grasp_recognized", &pcRecognition::graspRecognized, this);
  setTrainingServer = n.advertiseService("rail_recognition/set_training_mode", &pcRecognition::toggleTrainingMode, this);
  releaseServer = n.advertiseService("rail_recognition/drop_it_carl", &pcRecognition::releaseObject, this);
  
  readPointClouds();
}

bool pcRecognition::toggleTrainingMode(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  if (training == false)
  {
    training = true;
    epsilon = .95;
    ROS_INFO("Grasp training mode set to epsilon greedy.");
  }
  else
  {
    training = false;
    ROS_INFO("Grasp training mode disabled.");
  }
  
  ROS_INFO("Grasp statistics: ");
  for (unsigned int i = 0; i < successesList.size(); i ++)
  {
    for (unsigned int j = 0; j < successesList[i].size(); j ++)
    {
      ROS_INFO("Index: (%d,%d); Successes: %d; Attempts: %d", i, j, successesList[i][j], totalAttemptsList[i][j]);
    }
  }
  
  return true;
}

bool pcRecognition::releaseObject(rail_recognition::Release::Request &req, rail_recognition::Release::Response &res)
{
  geometry_msgs::PoseStamped releasePose;
  geometry_msgs::PoseStamped tempPose;
  releasePose.header.frame_id = req.releaseFrame;
  releasePose.pose = currentGrasp;
  releasePose.pose.position.z += .19;
  rail_grasping::RequestGrasp::Request releaseReq;
  rail_grasping::RequestGrasp::Response releaseRes;
  
  ROS_INFO("transforming pose");
  tfListener.transformPose("base_footprint", releasePose, tempPose);
  ROS_INFO("Pose transformed");
  releaseReq.graspPose = tempPose.pose;
  
  releasePosePublisher.publish(tempPose);
  
  ROS_INFO("Calling release service");
  requestReleaseClient.call(releaseReq, releaseRes);
  ROS_INFO("Release called");
  
  ROS_INFO("Finished release.");
}

bool pcRecognition::recognize(rail_segmentation::Recognize::Request &req,
    rail_segmentation::Recognize::Response &res)
{
  PointCloud<PointXYZRGB>::Ptr baseCloudPtr(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr targetCloudPtr(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);
  
  vector<geometry_msgs::Pose> baseGraspList;
  vector<geometry_msgs::Pose> targetGraspList;
  vector<geometry_msgs::Pose> resultGraspList;
  
  //convert point cloud to pcl format
  PCLPointCloud2 tempConvCloud;
  pcl_conversions::toPCL(req.objectCloud, tempConvCloud);
  fromPCLPointCloud2(tempConvCloud, *targetCloudPtr);

  //Filter input point cloud to remove noise, translate it to the origin for easier visualization
  filterCloudOutliers(targetCloudPtr, RADIUS, NUM_NEIGHBORS);
  translateToOrigin(targetCloudPtr, &targetGraspList);
  
  //Recognition
  float minScore = 999;
  int minIndex = 0;
  for (unsigned int j = 0; j < models.size(); j ++)
  {
    baseCloudPtr = models[j];

    float tempScore = scoreRegistration(baseCloudPtr, targetCloudPtr);
    if (tempScore < minScore)
    {
      minScore = tempScore;
      minIndex = j;
    }
  }
  
  if (minScore > .8)  //TODO: Adjust
  {
    ROS_INFO("Point cloud not recognized...");
    res.success = false;
    return true;
  }
  
  //Determine possible grasps
  targetGraspList.clear();
  baseGraspList = graspLists[minIndex];
  vector<geometry_msgs::Pose> finalGraspList;
  icpRegistration(models[minIndex], targetCloudPtr, baseGraspList, targetGraspList, &finalGraspList, false);
  //Store grasps in response with coordinate frame of original point cloud
  res.graspPoses.resize(targetGraspList.size());
  for (unsigned int i = 0; i < targetGraspList.size(); i ++)
  {
    res.graspPoses[i].header.frame_id = req.objectCloud.header.frame_id;
    res.graspPoses[i].pose = targetGraspList[i];
  }
  res.model = minIndex;
  //fill in object name
  //TODO: Have a better system for this!
  switch (minIndex)
  {
    case 0: res.name = "bowl"; break;
    case 1: res.name = "cup"; break;
    case 2: res.name = "fork"; break;
    default: res.name = "???"; break;
  }
  
  res.success = true;
  return true;
}

bool pcRecognition::recognizeAndPickup(rail_pick_and_place_msgs::RecognizeAndGrasp::Request &req, rail_pick_and_place_msgs::RecognizeAndGrasp::Response &res)
{
  ROS_INFO("\n");
  
  PointCloud<PointXYZRGB>::Ptr baseCloudPtr(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr targetCloudPtr(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);
  
  vector<geometry_msgs::Pose> baseGraspList;
  vector<geometry_msgs::Pose> targetGraspList;
  vector<geometry_msgs::Pose> resultGraspList;

  
  //make sure point cloud is in the correct frame for planning purposes
  //convert point cloud to pcl format
  sensor_msgs::PointCloud2 tempCloud;
  if (req.cloud.header.frame_id.compare("base_footprint") != 1)
  {
    sensor_msgs::PointCloud transformedCloud;
    tfListener.transformPointCloud("base_footprint", req.cloud, transformedCloud);
    sensor_msgs::convertPointCloudToPointCloud2(transformedCloud, tempCloud);
  }
  else
  {
    sensor_msgs::convertPointCloudToPointCloud2(req.cloud, tempCloud);
  }
  PCLPointCloud2 tempConvCloud;
  pcl_conversions::toPCL(tempCloud, tempConvCloud);
  //fromROSMsg(tempCloud, *targetCloudPtr);
  fromPCLPointCloud2(tempConvCloud, *targetCloudPtr);

  //Filter input point cloud to remove noise, translate it to the origin for easier visualization
  filterCloudOutliers(targetCloudPtr, RADIUS, NUM_NEIGHBORS);
  translateToOrigin(targetCloudPtr, &targetGraspList);
  
  //Recognition
  float minScore = 999;
  int minIndex = 0;
  for (unsigned int j = 0; j < models.size(); j ++)
  {
    baseCloudPtr = models[j];

    float tempScore = scoreRegistration(baseCloudPtr, targetCloudPtr);
    if (tempScore < minScore)
    {
      minScore = tempScore;
      minIndex = j;
    }
  }
  
  if (minScore > .8)  //TODO: Adjust
  {
    ROS_INFO("Point cloud not recognized...");
    res.success = false;
    return true;
  }
  
  res.objectIndex = minIndex;
  bool pickupObject = false;
  if (req.objectIndices.size() == 0) //model number doesn't matter
  {
    pickupObject = true;
  }
  else //compare to given object indices
  {
    for (unsigned int i = 0; i < req.objectIndices.size(); i ++)
    {
      if (minIndex == req.objectIndices[i])
      {
        pickupObject = true;
        break;
      }
    }
  }
  if (!pickupObject)
  {
    ROS_INFO("Object not recognized as one of the specified objects for pickup, moving on to the next object.");
    res.success = false;
    return true;
  }
  else
  {
    ROS_INFO("Specified object found, attempting grasp");
    res.success = true;
  }
  
  //ROS_INFO("Point cloud recognized as Model %d with score %f", minIndex + 1, minScore);
  
  //Determine possible grasps
  targetGraspList.clear();
  baseGraspList = graspLists[minIndex];
  vector<geometry_msgs::Pose> finalGraspList;
  icpRegistration(models[minIndex], targetCloudPtr, baseGraspList, targetGraspList, &finalGraspList, false);
  
  ROS_INFO("Determined %lu grasps", finalGraspList.size());
  
  chooseGrasp(minIndex, req.numAttempts, finalGraspList);
  
  return true;
}

bool pcRecognition::graspRecognized(rail_pick_and_place_msgs::GraspRecognized::Request &req, rail_pick_and_place_msgs::GraspRecognized::Response &res)
{
  //transform grasps to the base_footprint frame
  vector<geometry_msgs::Pose> transformedGrasps;
  transformedGrasps.resize(req.grasps.size());
  for (unsigned int i = 0; i < req.grasps.size(); i ++)
  {
    if (req.grasps[i].header.frame_id.compare("base_footrpint") == 0)
    {
      transformedGrasps[i] = req.grasps[i].pose;
    }
    else
    {
      geometry_msgs::PoseStamped tempPose;
      tfListener.transformPose("base_footprint", req.grasps[i], tempPose);
      transformedGrasps[i] = tempPose.pose;
    }
  }
  
  return this->chooseGrasp(req.objectIndex, req.numAttempts, transformedGrasps);
}

bool pcRecognition::chooseGrasp(int index, int numAttempts, vector<geometry_msgs::Pose> grasps)
{
  /******************************************************
   *********************  Training  ********************* 
   ******************************************************/
  if (training)
  {
    vector<float> successRate;
    vector<int> graspsAttempted;
    float totalChance = 0.0;
    
    //Calculate chance of grasp selection using epsilon-greedy selection
    float r = ((float)rand()) / ((float)RAND_MAX);
    ROS_INFO("r: %f", r);
    ROS_INFO("epsilon: %f", epsilon);
    if (r > 1 - epsilon)  //explore new grasps, i.e. select only grasps that have not been completed 3 times
    {
      ROS_INFO("\nGrasp chances: ");
      for (unsigned int j = 0; j < successesList[index].size(); j ++)
      {
        float chance = 1.0;
        if (totalAttemptsList[index][j] < 3)
        {
          successRate.push_back(chance);
          totalChance += chance;
        }
        else
        {
          chance = 0.0;
          successRate.push_back(chance);
          totalChance += chance;
          graspsAttempted.push_back(j);  //eliminate this grasp from being chosen
        }
    
        ROS_INFO("Index: %d; Chance: %f", j, chance);
      }
    }
    else  //select previously attempted grasps to refine success data
    {
      ROS_INFO("\nGrasp chances: ");
      for (unsigned int j = 0; j < successesList[index].size(); j ++)
      {
        float chance = 0.0;
        if (totalAttemptsList[index][j] < 3)
        {
          successRate.push_back(chance);
          totalChance += chance;
          graspsAttempted.push_back(j);  //eliminate this grasp from being chosen
        }
        else
        {
          chance = ((float)successesList[index][j]) / ((float)totalAttemptsList[index][j]);
          if (chance < .1)
            chance = .1;
          successRate.push_back(chance);
          totalChance += chance;
        }
    
        ROS_INFO("Index: %d; Chance: %f", j, chance);
      }
    }
  
    for (unsigned int i = 0; (int)i < numAttempts; i ++)
    {
      if (graspsAttempted.size() < grasps.size() && totalChance > 0.0)
      {
        ROS_INFO("Selecting grasp...");
        //probabilistically select a grasp based on success rate
        float r = 0.0;
        float sum = 0.0;
        //int graspIndex = successRate.size() - 1;
        int graspIndex = -1;
        while (graspAlreadyAttempted(graspIndex, graspsAttempted))
        {
          r = (((float)rand()) / ((float)RAND_MAX)) * totalChance;
          sum = 0.0;
          for (unsigned int j = 0; j < successRate.size(); j ++)
          {
            sum += successRate[j];
            if (sum >= r)
            {
              graspIndex = j;
              break;
            }
          }
        }

        ROS_INFO("Attempting grasp index %d at (%f, %f, %f)", graspIndex, grasps[graspIndex].position.x, grasps[graspIndex].position.y, grasps[graspIndex].position.z);
        graspsAttempted.push_back(graspIndex);
  
        rail_grasping::RequestGrasp srv;
        srv.request.graspPose = grasps[graspIndex];
        requestGraspClient.call(srv);

        if (srv.response.result)
        {  
          currentGrasp = graspLists[index][graspIndex];
          ROS_INFO("Grasp succeeded!");
          successesList[index][graspIndex] += 1;
          totalAttemptsList[index][graspIndex] += 1;
          break;
        }
        else
        {
          if (srv.response.earlyFailureDetected)
            ROS_INFO("Grasp index %d early failure detected....", graspIndex);
          else
          {
            ROS_INFO("Grasp index %d failed....", graspIndex);
            totalAttemptsList[index][graspIndex] += 1;
          }
        
        }
      }
      else
      {
        ROS_INFO("Ran out of grasps!");
      }
    }
    
    //Decay epsilon
    epsilon *= .975;
  }
  /******************************************************
   *********************  Testing  ********************** 
   ******************************************************/
  else
  {
    vector<float> successRate;
    vector<int> graspsAttempted;
    float totalChance = 0.0;
    //Calculate chance of grasp selection
    ROS_INFO("\nGrasp chances: ");
    for (unsigned int j = 0; j < successesList[index].size(); j ++)
    {
      float chance = 0.0;
      //assign chance of 0 if a specific grasp hasn't been trained
      if (totalAttemptsList[index][j] < 3)
      {
        successRate.push_back(chance);
        totalChance += chance;
        graspsAttempted.push_back(j);  //eliminate this grasp from being chosen
      }
      else
      {
        chance = ((float)successesList[index][j]) / ((float)totalAttemptsList[index][j]);
        //allow previously unsuccessful grasps a small chance at being selected again
        if (chance < .1)
          chance = .1;
        successRate.push_back(chance);
        totalChance += chance;
      }
    
      ROS_INFO("Index: %d; Chance: %f", j, chance);
    }
  
    for (unsigned int i = 0; (int)i < numAttempts; i ++)
    {
      if (graspsAttempted.size() < grasps.size())
      {
        ROS_INFO("Selecting grasp...");
        //select grasp with the highest proability of success
        //sort grasps by success rate
        vector<int> graspOrder;
        for (unsigned int j = 0; j < successRate.size(); j ++)
        {
          bool placed = false;
          for (unsigned int k = 0; k < graspOrder.size(); k ++)
          {
            if (successRate[j] > successRate[graspOrder[k]])
            {
              graspOrder.insert(graspOrder.begin() + k, j);
              placed = true;
              break;
            }
          }
          if (!placed)
            graspOrder.push_back(j);
        }
        
        int pos = 0;
        int graspIndex = graspOrder[pos];
        while (graspAlreadyAttempted(graspIndex, graspsAttempted))
        {
          pos ++;
          graspIndex = graspOrder[pos];
        }

        ROS_INFO("Attempting grasp index %d at (%f, %f, %f)", graspIndex, grasps[graspIndex].position.x, grasps[graspIndex].position.y, grasps[graspIndex].position.z);
        graspsAttempted.push_back(graspIndex);
  
        rail_grasping::RequestGrasp srv;
        srv.request.graspPose = grasps[graspIndex];
        requestGraspClient.call(srv);

        if (srv.response.result)
        {  
          currentGrasp = graspLists[index][graspIndex];
          ROS_INFO("Grasp succeeded!");
          successesList[index][graspIndex] += 1;
          totalAttemptsList[index][graspIndex] += 1;
          break;
        }
        else
        {
          if (srv.response.earlyFailureDetected)
            ROS_INFO("Grasp index %d early failure detected....", graspIndex);
          else
          {
            ROS_INFO("Grasp index %d failed....", graspIndex);
            totalAttemptsList[index][graspIndex] += 1;
          }
        
        }
      }
      else
      {
        ROS_INFO("Ran out of grasps!");
      }
    }
  }
  
  return true;
}

bool pcRecognition::graspAlreadyAttempted(int index, vector<int> list)
{
  if (index < 0)
    return true;
  
  for (unsigned int i = 0; i < list.size(); i ++)
  {
    if (list[i] == index)
      return true;
  }
  
  return false;
}

bool pcRecognition::getCloud(string filename, PointCloud<PointXYZRGB>::Ptr pointcloudOut, vector<geometry_msgs::Pose> *graspListOut, vector<int> *successesListOut, vector<int> *totalAttemptsOut)
{  
  rail_recognition::ReadGrasp srv;
  srv.request.grasp_entry = filename;
  srv.request.visualize_grasps = false;

  readGraspClient.call(srv);

  if (srv.response.success)
  {  
    sensor_msgs::convertPointCloudToPointCloud2(srv.response.grasp.pointCloud, baseCloud);
    PCLPointCloud2 tempCloud;
    pcl_conversions::toPCL(baseCloud, tempCloud);
    //fromROSMsg(baseCloud, *pointcloudOut);
    fromPCLPointCloud2(tempCloud, *pointcloudOut);
    
    *graspListOut = srv.response.grasp.gripperPoses;
    *successesListOut = srv.response.grasp.successes;
    *totalAttemptsOut = srv.response.grasp.totalAttempts;
  }

  return srv.response.success;  
}

void pcRecognition::readPointClouds()
{  
  ROS_INFO("Reading models...");
  bool reading = true;
  stringstream ss;
  int count = 1;
  while (reading)
  {
    /*
    ss.str("");
    ss << "model_" << count << ".pcd";
    PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
    if (io::loadPCDFile<PointXYZRGB>(ss.str(), *tempCloudPtr) == -1)
    {
      reading = false;
      count --;
    }
    else
    {
      models.push_back(tempCloudPtr);
      count ++;
    }
    */
    
    ss.str("");
    ss << "model_" << count << ".txt";
    PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
    vector<geometry_msgs::Pose> tempGraspList;
    vector<int> tempSuccesses;
    vector<int> tempTotalAttempts;
    reading = getCloud(ss.str(), tempCloudPtr, &tempGraspList, &tempSuccesses, &tempTotalAttempts);
    if (reading)
    {
      models.push_back(tempCloudPtr);
      graspLists.push_back(tempGraspList);
      successesList.push_back(tempSuccesses);
      totalAttemptsList.push_back(tempTotalAttempts);
      
      count ++;
    }
    else
      count --;
  }
  ROS_INFO("Read %d Models", count);
}

float pcRecognition::scoreRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{
  IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
  icp.setInputSource(targetCloudPtr);
  icp.setInputTarget(baseCloudPtr);
  PointCloud<PointXYZRGB>::Ptr targetTransformedPtr(new PointCloud<PointXYZRGB>);
  icp.align(*targetTransformedPtr);
  //float icpScore = icp.getFitnessScore();
  //ROS_INFO_STREAM("ICP convergence score: " << icpScore);
  
  float dstError = calculateRegistrationMetricDstError(baseCloudPtr, targetTransformedPtr);
  float colorError = calculateRegistrationMetricOverlap(baseCloudPtr, targetTransformedPtr, .005);
  //float avgColorDiff = calculateRegistrationMetricColorRange(baseCloudPtr, targetTransformedPtr);
  //float maxDstDiff = calculateRegistrationMetricDistance(baseCloudPtr, targetTransformedPtr);
  //ROS_INFO("Calculated distance error score: %f", dstError);
  //ROS_INFO("Calculated overlap score: %f", overlap);
  
  float result = ALPHA * (3*dstError) + (1 - ALPHA) * (colorError/100.0);
  
  return result;
}

PointCloud<PointXYZRGB>::Ptr pcRecognition::icpRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, vector<geometry_msgs::Pose> baseGrasps, vector<geometry_msgs::Pose> targetGrasps, vector<geometry_msgs::Pose> *resultGrasps, bool scoreFiltered)
{
  //Determine which point cloud is larger, and use that as the base point cloud
  bool swapped = false;
  if (targetCloudPtr->size() > baseCloudPtr->size())
  {
    PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
    tempCloudPtr = baseCloudPtr;
    baseCloudPtr = targetCloudPtr;
    targetCloudPtr = tempCloudPtr;
    vector<geometry_msgs::Pose> tempBaseGrasps(baseGrasps);
    vector<geometry_msgs::Pose> tempTargetGrasps(targetGrasps);
    baseGrasps = tempTargetGrasps;
    targetGrasps = tempBaseGrasps;
    swapped = true;
  }

  IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
  icp.setInputSource(targetCloudPtr);
  icp.setInputTarget(baseCloudPtr);
  PointCloud<PointXYZRGB>::Ptr targetTransformedPtr(new PointCloud<PointXYZRGB>);
  icp.align(*targetTransformedPtr);
  float icpScore = icp.getFitnessScore();

  //debug:
  //ROS_INFO_STREAM("ICP convergence score: " << icpScore);
  //float dstError = calculateRegistrationMetricDstError(baseCloudPtr, targetTransformedPtr);
  //float overlap = calculateRegistrationMetricOverlap(baseCloudPtr, targetTransformedPtr, .005);
  //float avgColorDiff = calculateRegistrationMetricColorRange(baseCloudPtr, targetTransformedPtr);
  //float maxDstDiff = calculateRegistrationMetricDistance(baseCloudPtr, targetTransformedPtr);
  //ROS_INFO("Calculated distance error score: %f", dstError);
  //ROS_INFO("Calculated overlap score: %f", overlap);
  
  PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>& result = *resultPtr;
  
  if (scoreFiltered)
  {
    if (icpScore > .00004)
    {
      return resultPtr;
    }
    else
    {
      result = *baseCloudPtr + *targetTransformedPtr;
  
      filterRedundentPoints(resultPtr, DST_THRESHOLD);
    }
  }
  else
  {
    //Transform grasps to the appropriate position and orientation
    Eigen::Matrix4f transform = icp.getFinalTransformation();
    tf::Matrix3x3 rotationMatrix(  transform(0,0), transform(0,1), transform(0,2),
                  transform(1,0), transform(1,1), transform(1,2),
                  transform(2,0), transform(2,1), transform(2,2));
    tf::Transform tfTransform;
    tf::Quaternion quat;
    rotationMatrix.getRotation(quat);
    tfTransform.setOrigin(tf::Vector3(transform(0,3), transform(1,3), transform(2,3)));
    tfTransform.setRotation(quat);
    
    ros::Time now = ros::Time::now();
    tfBroadcaster.sendTransform(tf::StampedTransform(tfTransform, now, "target_cloud_frame", "base_cloud_frame"));
    tfListener.waitForTransform("target_cloud_frame", "base_cloud_frame", now, ros::Duration(5.0));
    
    if (swapped)
    {
      for (unsigned int i = 0; i < targetGrasps.size(); i ++)
      {
        geometry_msgs::PoseStamped poseOut;
        geometry_msgs::PoseStamped tempPoseStamped;
        tempPoseStamped.pose = targetGrasps[i];
        tempPoseStamped.header.stamp = now;
        tempPoseStamped.header.frame_id = "target_cloud_frame";
      
        tfListener.transformPose("base_cloud_frame", tempPoseStamped, poseOut);
      
        //undo origin translation
        poseOut.pose.position.x += xTrans;
        poseOut.pose.position.y += yTrans;
        poseOut.pose.position.z += zTrans;
      
        targetGrasps[i] = poseOut.pose;
      }
    }
    else
    {
      for (unsigned int i = 0; i < baseGrasps.size(); i ++)
      {
        geometry_msgs::PoseStamped poseOut;
        geometry_msgs::PoseStamped tempPoseStamped;
        tempPoseStamped.pose = baseGrasps[i];
        tempPoseStamped.header.stamp = now;
        tempPoseStamped.header.frame_id = "base_cloud_frame";
      
        tfListener.transformPose("target_cloud_frame", tempPoseStamped, poseOut);
      
        //undo origin translation
        poseOut.pose.position.x += xTrans;
        poseOut.pose.position.y += yTrans;
        poseOut.pose.position.z += zTrans;
      
        baseGrasps[i] = poseOut.pose;
      }
    }
    
    //merge point clouds
    result = *baseCloudPtr + *targetTransformedPtr;
    
    //merge grasp lists
    for (unsigned int i = 0 ; i < baseGrasps.size(); i ++)
    {
      (*resultGrasps).push_back(baseGrasps[i]);
    }
    for (unsigned int i = 0; i < targetGrasps.size(); i ++)
    {
      (*resultGrasps).push_back(targetGrasps[i]);
    }
  
    filterRedundentPoints(resultPtr, DST_THRESHOLD);
  }
  
  //classifyMerge(overlap, maxDstDiff, dstError, avgColorDiff);
  
  //return targetTransformedPtr;
  return resultPtr;
}


float pcRecognition::calculateRegistrationMetricDstError(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{  
  float score = 0;
  KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
  searchTree.setInputCloud(baseCloudPtr);
  vector<int> removeIndices;
  vector<int> indices;
  vector<float> distances;
  
  for (unsigned int i = 0; i < targetCloudPtr->size(); i ++)
  {
    searchTree.nearestKSearch(targetCloudPtr->at(i), 1, indices, distances);
    score += distances[0];
  }
  
  return score;
}

float pcRecognition::calculateRegistrationMetricOverlap(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, float dstThreshold)
{
  float score = 0;
  float colorError = 0;
  KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
  searchTree.setInputCloud(baseCloudPtr);
  vector<int> removeIndices;
  vector<int> indices;
  vector<float> distances;
  
  for (unsigned int i = 0; i < targetCloudPtr->size(); i ++)
  {
    PointXYZRGB searchPoint = targetCloudPtr->at(i);
    int neighbors = searchTree.radiusSearch(searchPoint, dstThreshold, indices, distances);
    if (neighbors > 0)
    {
      score ++;
      
      float colorDistance = 0;
      for (unsigned int j = 0; j < indices.size(); j ++)
      {
        PointXYZRGB point = baseCloudPtr->at(indices[j]);
        colorDistance += sqrt(pow(searchPoint.r - point.r, 2) + pow(searchPoint.g - point.g, 2) + pow(searchPoint.b - point.b, 2));
      }
      colorDistance /= neighbors;
      colorError += colorDistance;
    }
  }
  
  colorError /= score;  
  score /= targetCloudPtr->size();

  //debug:
  //ROS_INFO("Color Error: %f", colorError);
  //ROS_INFO("Overlap Score: %f", score);

  return colorError;
}

float pcRecognition::calculateRegistrationMetricColorRange(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{
  float avgr = 0, avgg = 0, avgb = 0;
  
  for (unsigned int i = 0; i < targetCloudPtr->size(); i ++)
  {
    PointXYZRGB point = targetCloudPtr->at(i);
      
    avgr += point.r;
    avgg += point.g;
    avgb += point.b;
  }

  //debug:
  //ROS_INFO("Target AVG Red: %f", avgr /= targetCloudPtr->size());
  //ROS_INFO("Target AVG Green: %f", avgg /= targetCloudPtr->size());
  //ROS_INFO("Target AVG Blue: %f", avgb /= targetCloudPtr->size());
  
  float avg1 = (avgr + avgg + avgb) / targetCloudPtr->size();
  
  avgr = 0;
  avgg = 0;
  avgb = 0;
    
  for (unsigned int i = 0; i < baseCloudPtr->size(); i ++)
  {
    PointXYZRGB point = baseCloudPtr->at(i);
      
    avgr += point.r;
    avgg += point.g;
    avgb += point.b;
  }

  //debug:
  //ROS_INFO("Base AVG Red: %f", avgr /= targetCloudPtr->size());
  //ROS_INFO("Base AVG Green: %f", avgg /= targetCloudPtr->size());
  //ROS_INFO("Base AVG Blue: %f", avgb /= targetCloudPtr->size());
  
  float avg2 = (avgr + avgg + avgb) / baseCloudPtr->size();
  
  return fabs(avg1 - avg2);
}

float pcRecognition::calculateRegistrationMetricDistance(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{
  float maxDst = 0;
  
  for (unsigned int i = 0; i < targetCloudPtr->size() - 1; i ++)
  {
    for (unsigned int j = 1; j < targetCloudPtr->size(); j ++)
    {
      PointXYZRGB p1 = targetCloudPtr->at(i);
      PointXYZRGB p2 = targetCloudPtr->at(j);
      
      float dst = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
      if (dst > maxDst)
        maxDst = dst;
    }
  }

  //debug:
  //ROS_INFO("Max distance for target: %f", maxDst);  
  
  float maxDst2 = 0;
  
  for (unsigned int i = 0; i < baseCloudPtr->size() - 1; i ++)
  {
    for (unsigned int j = 1; j < baseCloudPtr->size(); j ++)
    {
      PointXYZRGB p1 = baseCloudPtr->at(i);
      PointXYZRGB p2 = baseCloudPtr->at(j);
      
      float dst = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
      if (dst > maxDst2)
        maxDst2 = dst;
    }
  }

  //debug:
  //ROS_INFO("Max distance for base: %f", maxDst3);
  
  return fabs(maxDst - maxDst2);
}

void pcRecognition::filterCloudOutliers(PointCloud<PointXYZRGB>::Ptr cloudPtr, double radius, int numNeighborThreshold)
{
  KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
  searchTree.setInputCloud(cloudPtr);
  vector<int> removeIndices;
  vector<int> indices;
  vector<float> distances;
  
  for (unsigned int i = 0; i < cloudPtr->size(); i ++)
  {
    int neighbors = searchTree.radiusSearch(cloudPtr->at(i), radius, indices, distances);
    if (neighbors < numNeighborThreshold)
      removeIndices.push_back(i);
  }
  
  sort(removeIndices.begin(), removeIndices.end());
  reverse(removeIndices.begin(), removeIndices.end());
  
  ROS_INFO("Found %lu points to filter", removeIndices.size());
  
  for (int i = (int)(removeIndices.size()) - 1; i >= 0; i --)
  {
    cloudPtr->erase(cloudPtr->begin() + i);
  }
  
}

void pcRecognition::filterRedundentPoints(PointCloud<PointXYZRGB>::Ptr cloudPtr, double dstThreshold)
{
  KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
  searchTree.setInputCloud(cloudPtr);
  vector<int> removeIndices;
  vector<int> indices;
  vector<float> distances;
  
  for (int i = (int)(cloudPtr->size()) - 1; i >= 0; i --)
  {
    int neighbors = searchTree.radiusSearch(cloudPtr->at(i), dstThreshold, indices, distances);
    if (neighbors > 1)
      cloudPtr->erase(cloudPtr->begin() + i);
  }
}

void pcRecognition::translateToOrigin(PointCloud<PointXYZRGB>::Ptr cloudPtr, vector<geometry_msgs::Pose> *grasps)
{
  float x = 0;
  float y = 0;
  float z = 0;
  
  for (unsigned int i = 0; i < cloudPtr->size(); i ++)
  {
    x += cloudPtr->at(i).x;
    y += cloudPtr->at(i).y;
    z += cloudPtr->at(i).z;
  }
  x /= cloudPtr->size();
  y /= cloudPtr->size();
  z /= cloudPtr->size();
  
  Eigen::Matrix4f transform;
  transform << 1, 0, 0, -x,
         0, 1, 0, -y,
         0, 0, 1, -z,
         0, 0, 0, 1;
  
  transformPointCloud(*cloudPtr, *cloudPtr, transform);
    
  //transform grasps
  xTrans = x;
  yTrans = y;
  zTrans = z;
  for (unsigned int i = 0; i < (*grasps).size(); i ++)
  {
    (*grasps)[i].position.x -= x;
    (*grasps)[i].position.y -= y;
    (*grasps)[i].position.z -= z;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_recognition");
  
  pcRecognition pcr;
  
  ros::spin();
  
  return 0;
}
