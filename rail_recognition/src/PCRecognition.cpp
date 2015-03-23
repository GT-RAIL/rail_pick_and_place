#include <rail_recognition/PCRecognition.h>

using namespace std;
using namespace pcl;
using namespace rail::pick_and_place;

PCRecognition::PCRecognition() :
    asRecognizeObject(n, "pc_recognition/recognize_object", boost::bind(&PCRecognition::executeRecognizeObject, this, _1), false),
    asRecognizeObjectByName(n, "pc_recognition/recognize_object_by_name", boost::bind(&PCRecognition::executeRecognizeObjectByName, this, _1), false),
    asRecognizeObjects(n, "pc_recognition/recognize_objects", boost::bind(&PCRecognition::executeRecognizeObjects, this, _1), false)
{
  //setup connection to grasp database
  // set defaults
  int port = graspdb::Client::DEFAULT_PORT;
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");

  // grab any parameters we need
  n.getParam("/graspdb/host", host);
  n.getParam("/graspdb/port", port);
  n.getParam("/graspdb/user", user);
  n.getParam("/graspdb/password", password);
  n.getParam("/graspdb/db", db);

  // connect to the grasp database
  graspdb = new graspdb::Client(host, port, user, password, db);
  bool okay = graspdb->connect();

  if (okay)
    ROS_INFO("Successfully connected to grasp database.");
  else
    ROS_INFO("Could not connect to grasp database.");

  xTrans = 0.0;
  yTrans = 0.0;
  zTrans = 0.0;

  asRecognizeObject.start();
  asRecognizeObjectByName.start();
  asRecognizeObjects.start();
}

void PCRecognition::executeRecognizeObject(const rail_manipulation_msgs::RecognizeObjectGoalConstPtr &goal)
{
  //populate candidates
  vector<string> names;
  graspdb->getUniqueGraspModelObjectNames(names);
  vector<graspdb::GraspModel> candidates;
  for (unsigned int i = 0; i < names.size(); i ++)
  {
    vector<graspdb::GraspModel> tempCandidates;
    graspdb->loadGraspModelsByObjectName(names[i], tempCandidates);
    candidates.insert(candidates.end(), tempCandidates.begin(), tempCandidates.end());
  }

  //perform recognition
  rail_manipulation_msgs::RecognizeObjectResult result;
  result.object = goal->object;
  if (!recognizeObject(&result.object, candidates))
  {
    ROS_INFO("Object could not be recognized.");
  }
  else
  {
    ROS_INFO("Object successfully recognized!");
  }
  asRecognizeObject.setSucceeded(result);
}

void PCRecognition::executeRecognizeObjectByName(const rail_manipulation_msgs::RecognizeObjectByNameGoalConstPtr &goal)
{
  //populate candidates
  vector<graspdb::GraspModel> candidates;
  graspdb->loadGraspModelsByObjectName(goal->name, candidates);

  //perform recognition
  rail_manipulation_msgs::RecognizeObjectByNameResult result;
  result.object = goal->object;
  if (!recognizeObject(&result.object, candidates))
  {
    ROS_INFO("Object could not be recognized.");
  }
  else
  {
    ROS_INFO("Object successfully recognized!");
  }
  asRecognizeObjectByName.setSucceeded(result);
}

void PCRecognition::executeRecognizeObjects(const rail_manipulation_msgs::RecognizeObjectsGoalConstPtr &goal)
{
  //populate candidates
  vector<string> names;
  graspdb->getUniqueGraspModelObjectNames(names);
  vector<graspdb::GraspModel> candidates;
  for (unsigned int i = 0; i < names.size(); i ++)
  {
    vector<graspdb::GraspModel> tempCandidates;
    graspdb->loadGraspModelsByObjectName(names[i], tempCandidates);
    candidates.insert(candidates.end(), tempCandidates.begin(), tempCandidates.end());
  }

  //perform recognition
  rail_manipulation_msgs::RecognizeObjectsResult result;
  result.objects = goal->objects;
  for (unsigned int i = 0; i < result.objects.objects.size(); i ++)
  {
    if (!recognizeObject(&result.objects.objects[i], candidates))
    {
      ROS_INFO("Object %d could not be recognized.", i);
    }
    else
    {
      ROS_INFO("Object %d successfully recognized!", i);
    }
  }
  asRecognizeObjects.setSucceeded(result);
}

bool PCRecognition::recognizeObject(rail_manipulation_msgs::SegmentedObject *object, std::vector<graspdb::GraspModel> candidates)
{
  //convert object and candidates to Model objects (pcl-friendly point clouds)
  Model model;
  model.copyFromSegmentedObjectMsg(*object);
  vector<Model> candidateModels;
  candidateModels.resize(candidates.size());
  for (unsigned int i = 0; i < candidateModels.size(); i ++)
  {
    candidateModels[i].copyFromGraspModelMsg(candidates[i].toROSGraspModelMessage());
  }

  //preprocess input cloud
  filterCloudOutliers(model.pointCloud, RADIUS, NUM_NEIGHBORS);
  translateToOrigin(model.pointCloud, &model.graspList);

  //perform recognition
  float minScore = 999;
  int minIndex = 0;
  for (unsigned int i = 0; i < candidateModels.size(); i ++)
  {
    float tempScore = scoreRegistration(candidateModels[i].pointCloud, model.pointCloud);
    if (tempScore < minScore)
    {
      minScore = tempScore;
      minIndex = i;
    }
  }

  if (minScore > .8)
  {
    ROS_INFO("Point cloud not recognized...");
    return false;
  }

  //Determine possible grasps
  vector<rail_pick_and_place_msgs::GraspWithSuccessRate> targetGraspList;
  vector<rail_pick_and_place_msgs::GraspWithSuccessRate> baseGraspList = candidateModels[minIndex].graspList;
  vector<rail_pick_and_place_msgs::GraspWithSuccessRate> finalGraspList;
  icpRegistration(candidateModels[minIndex].pointCloud, model.pointCloud, baseGraspList, targetGraspList, &finalGraspList);
  //Store grasps in response with coordinate frame of original point cloud
  object->grasps.clear();
  vector<float> successRates;
  for (unsigned int i = 0; i < finalGraspList.size(); i ++)
  {
    //add grasps if they have a non-zero success rate, sorted by success rate
    if (finalGraspList[i].successes > 0)
    {
      bool inserted = false;
      float successRate = ((float)finalGraspList[i].successes) / ((float)finalGraspList[i].attempts);
      for (unsigned int j = 0; j < successRates.size(); j ++)
      {
        if (successRate < successRates[j])
        {
          object->grasps.insert(object->grasps.begin() + j, finalGraspList[i].grasp_pose);
          successRates.insert(successRates.begin() + j, successRate);
          inserted = true;
          break;
        }
      }
      if (!inserted)
      {
        object->grasps.push_back(finalGraspList[i].grasp_pose);
        successRates.push_back(successRate);
      }
    }
  }

  //fill in other recognition information
  object->name = candidateModels[minIndex].objectName;
  object->model_id = candidateModels[minIndex].modelID;
  object->recognized = true;

  return true;
}

float PCRecognition::scoreRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
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

  float result = ALPHA * (3 * dstError) + (1 - ALPHA) * (colorError / 100.0);

  return result;
}

PointCloud<PointXYZRGB>::Ptr PCRecognition::icpRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr,
    vector<rail_pick_and_place_msgs::GraspWithSuccessRate> baseGrasps,
    vector<rail_pick_and_place_msgs::GraspWithSuccessRate> targetGrasps,
    vector<rail_pick_and_place_msgs::GraspWithSuccessRate> *resultGrasps)
{
  //Determine which point cloud is larger, and use that as the base point cloud
  bool swapped = false;
  if (targetCloudPtr->size() > baseCloudPtr->size())
  {
    PointCloud<PointXYZRGB>::Ptr tempCloudPtr(new PointCloud<PointXYZRGB>);
    tempCloudPtr = baseCloudPtr;
    baseCloudPtr = targetCloudPtr;
    targetCloudPtr = tempCloudPtr;
    vector<rail_pick_and_place_msgs::GraspWithSuccessRate> tempBaseGrasps(baseGrasps);
    vector<rail_pick_and_place_msgs::GraspWithSuccessRate> tempTargetGrasps(targetGrasps);
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
  PointCloud<PointXYZRGB> &result = *resultPtr;

  //Transform grasps to the appropriate position and orientation
  Eigen::Matrix4f transform = icp.getFinalTransformation();
  tf::Matrix3x3 rotationMatrix(transform(0, 0), transform(0, 1), transform(0, 2),
      transform(1, 0), transform(1, 1), transform(1, 2),
      transform(2, 0), transform(2, 1), transform(2, 2));
  tf::Transform tfTransform;
  tf::Quaternion quat;
  rotationMatrix.getRotation(quat);
  tfTransform.setOrigin(tf::Vector3(transform(0, 3), transform(1, 3), transform(2, 3)));
  tfTransform.setRotation(quat);

  ros::Time now = ros::Time::now();
  tfBroadcaster.sendTransform(tf::StampedTransform(tfTransform, now, "base_cloud_frame", "target_cloud_frame"));
  tfListener.waitForTransform("base_cloud_frame", "target_cloud_frame", now, ros::Duration(5.0));

  if (swapped)
  {
    for (unsigned int i = 0; i < targetGrasps.size(); i++)
    {
      geometry_msgs::PoseStamped poseOut;
      geometry_msgs::PoseStamped tempPoseStamped;
      tempPoseStamped.pose = targetGrasps[i].grasp_pose.pose;
      tempPoseStamped.header.stamp = now;
      tempPoseStamped.header.frame_id = "target_cloud_frame";

      tfListener.transformPose("base_cloud_frame", tempPoseStamped, poseOut);

      //undo origin translation
      poseOut.pose.position.x += xTrans;
      poseOut.pose.position.y += yTrans;
      poseOut.pose.position.z += zTrans;

      targetGrasps[i].grasp_pose.pose = poseOut.pose;
    }
  }
  else
  {
    for (unsigned int i = 0; i < baseGrasps.size(); i++)
    {
      geometry_msgs::PoseStamped poseOut;
      geometry_msgs::PoseStamped tempPoseStamped;
      tempPoseStamped.pose = baseGrasps[i].grasp_pose.pose;
      tempPoseStamped.header.stamp = now;
      tempPoseStamped.header.frame_id = "base_cloud_frame";

      tfListener.transformPose("target_cloud_frame", tempPoseStamped, poseOut);

      //undo origin translation
      poseOut.pose.position.x += xTrans;
      poseOut.pose.position.y += yTrans;
      poseOut.pose.position.z += zTrans;

      baseGrasps[i].grasp_pose.pose = poseOut.pose;
    }
  }

  //merge point clouds
  result = *baseCloudPtr + *targetTransformedPtr;

  //merge grasp lists
  for (unsigned int i = 0; i < baseGrasps.size(); i++)
  {
    (*resultGrasps).push_back(baseGrasps[i]);
  }
  for (unsigned int i = 0; i < targetGrasps.size(); i++)
  {
    (*resultGrasps).push_back(targetGrasps[i]);
  }

  filterRedundentPoints(resultPtr, DST_THRESHOLD);

  return resultPtr;
}

float PCRecognition::calculateRegistrationMetricDstError(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{
  float score = 0;
  KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
  searchTree.setInputCloud(baseCloudPtr);
  vector<int> removeIndices;
  vector<int> indices;
  vector<float> distances;

  for (unsigned int i = 0; i < targetCloudPtr->size(); i++)
  {
    searchTree.nearestKSearch(targetCloudPtr->at(i), 1, indices, distances);
    score += distances[0];
  }

  return score;
}

float PCRecognition::calculateRegistrationMetricOverlap(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, float dstThreshold)
{
  float score = 0;
  float colorError = 0;
  KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
  searchTree.setInputCloud(baseCloudPtr);
  vector<int> removeIndices;
  vector<int> indices;
  vector<float> distances;

  for (unsigned int i = 0; i < targetCloudPtr->size(); i++)
  {
    PointXYZRGB searchPoint = targetCloudPtr->at(i);
    int neighbors = searchTree.radiusSearch(searchPoint, dstThreshold, indices, distances);
    if (neighbors > 0)
    {
      score++;

      float colorDistance = 0;
      for (unsigned int j = 0; j < indices.size(); j++)
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

float PCRecognition::calculateRegistrationMetricColorRange(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{
  float avgr = 0, avgg = 0, avgb = 0;

  for (unsigned int i = 0; i < targetCloudPtr->size(); i++)
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

  for (unsigned int i = 0; i < baseCloudPtr->size(); i++)
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

float PCRecognition::calculateRegistrationMetricDistance(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{
  float maxDst = 0;

  for (unsigned int i = 0; i < targetCloudPtr->size() - 1; i++)
  {
    for (unsigned int j = 1; j < targetCloudPtr->size(); j++)
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

  for (unsigned int i = 0; i < baseCloudPtr->size() - 1; i++)
  {
    for (unsigned int j = 1; j < baseCloudPtr->size(); j++)
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

void PCRecognition::filterCloudOutliers(PointCloud<PointXYZRGB>::Ptr cloudPtr, double radius, int numNeighborThreshold)
{
  KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
  searchTree.setInputCloud(cloudPtr);
  vector<int> removeIndices;
  vector<int> indices;
  vector<float> distances;

  for (unsigned int i = 0; i < cloudPtr->size(); i++)
  {
    int neighbors = searchTree.radiusSearch(cloudPtr->at(i), radius, indices, distances);
    if (neighbors < numNeighborThreshold)
      removeIndices.push_back(i);
  }

  sort(removeIndices.begin(), removeIndices.end());
  reverse(removeIndices.begin(), removeIndices.end());

  ROS_INFO("Found %lu points to filter", removeIndices.size());

  for (int i = (int) (removeIndices.size()) - 1; i >= 0; i--)
  {
    cloudPtr->erase(cloudPtr->begin() + i);
  }

}

void PCRecognition::filterRedundentPoints(PointCloud<PointXYZRGB>::Ptr cloudPtr, double dstThreshold)
{
  KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
  searchTree.setInputCloud(cloudPtr);
  vector<int> removeIndices;
  vector<int> indices;
  vector<float> distances;

  for (int i = (int) (cloudPtr->size()) - 1; i >= 0; i--)
  {
    int neighbors = searchTree.radiusSearch(cloudPtr->at(i), dstThreshold, indices, distances);
    if (neighbors > 1)
      cloudPtr->erase(cloudPtr->begin() + i);
  }
}

void PCRecognition::translateToOrigin(PointCloud<PointXYZRGB>::Ptr cloudPtr, vector<rail_pick_and_place_msgs::GraspWithSuccessRate> *grasps)
{
  Eigen::Vector4f centroid;
  compute3DCentroid(*cloudPtr, centroid);
  float x = centroid[0];
  float y = centroid[1];
  float z = centroid[2];

  //transform point cloud
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
  for (unsigned int i = 0; i < grasps->size(); i++)
  {
    grasps->at(i).grasp_pose.pose.position.x -= x;
    grasps->at(i).grasp_pose.pose.position.y -= y;
    grasps->at(i).grasp_pose.pose.position.z -= z;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_recognition");

  PCRecognition pcr;

  ros::spin();

  return 0;
}
