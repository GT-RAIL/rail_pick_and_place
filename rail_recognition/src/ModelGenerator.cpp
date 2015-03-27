#include <rail_recognition/ModelGenerator.h>

using namespace std;
using namespace pcl;
using namespace rail::pick_and_place;

ModelGenerator::ModelGenerator() :
    asGenerateModels(n, "pc_registration/generate_models", boost::bind(&ModelGenerator::executeGenerateModels, this, _1), false)
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

  asGenerateModels.start();
}

void ModelGenerator::executeGenerateModels(const rail_pick_and_place_msgs::GenerateModelsGoalConstPtr &goal)
{
  rail_pick_and_place_msgs::GenerateModelsResult result;
  rail_pick_and_place_msgs::GenerateModelsFeedback feedback;

  feedback.message = "Populating model generation graph...";
  asGenerateModels.publishFeedback(feedback);
  vector<Model> models;
  models.resize(goal->individualGraspModelIds.size() + goal->mergedModelIds.size());
  for (unsigned int i = 0; i < goal->individualGraspModelIds.size(); i ++)
  {
    graspdb::GraspDemonstration demonstration;
    graspdb->loadGraspDemonstration(goal->individualGraspModelIds[i], demonstration);
    models[i].copyFromGraspDemonstrationMsg(demonstration.toROSGraspDemonstrationMessage());
  }
  for (unsigned int i = 0; i < goal->mergedModelIds.size(); i ++)
  {
    graspdb::GraspModel model;
    graspdb->loadGraspModel(goal->mergedModelIds[i], model);
    models[goal->individualGraspModelIds.size() + i].copyFromGraspModelMsg(model.toROSGraspModelMessage());
  }

  feedback.message = "Registering models, please wait...";
  asGenerateModels.publishFeedback(feedback);
  result.newModelsTotal = registerPointCloudsGraph(models, goal->maxModelSize, result.unusedModelIds);

  feedback.message = "Model generation complete.";
  asGenerateModels.publishFeedback(feedback);
  asGenerateModels.setSucceeded(result);
}

int ModelGenerator::registerPointCloudsGraph(vector<Model> models, int maxModelSize, vector<int> unusedModelIds)
{
  PointCloud<PointXYZRGB>::Ptr baseCloudPtr(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr targetCloudPtr(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);

  vector<rail_pick_and_place_msgs::GraspWithSuccessRate> baseGraspList;
  vector<rail_pick_and_place_msgs::GraspWithSuccessRate> targetGraspList;
  vector<rail_pick_and_place_msgs::GraspWithSuccessRate> resultGraspList;

  //Filter point clouds to remove noise, translate them to the origin for easier visualization
  for (unsigned int i = 0; i < models.size(); i++)
  {
    filterCloudOutliers(models[i].pointCloud, RADIUS, NUM_NEIGHBORS);
  }

  bool finished = false;

  while (!finished)
  {
    for (unsigned int i = 0; i < models.size(); i++)
    {
      translateToOrigin(models[i].pointCloud, &models[i].graspList);
    }
    finished = true;
    if (models.size() > 1)
    {
      random_shuffle(models.begin(), models.end());
      for (int i = (int)(models.size()) - 1; i >= 1; i--)
      {
        if (i < (int)(models.size()))
        {
          for (int j = i - 1; j >= 0; j--)
          {
            if (models[i].graspList.size() + models[j].graspList.size() > maxModelSize)
            {
              ROS_INFO("Skipping pair %d-%d as a merge would exceed the maximum model size.", i, j);
              continue;
            }
            else
            {
              ROS_INFO("Checking pair %d-%d", i, j);
            }

            baseCloudPtr = models[i].pointCloud;
            targetCloudPtr = models[j].pointCloud;

            if (checkRegistration(baseCloudPtr, targetCloudPtr))
            {
              ROS_INFO("Registration successful!");

              baseGraspList = models[i].graspList;
              targetGraspList = models[j].graspList;
              resultGraspList.clear();
              resultPtr = icpRegistration(baseCloudPtr, targetCloudPtr, baseGraspList, targetGraspList, &resultGraspList);
              filterRedundentPoints(resultPtr, DST_THRESHOLD);

              Model mergedModel;
              mergedModel.pointCloud = resultPtr;
              mergedModel.graspList = resultGraspList;
              mergedModel.objectName = models[i].objectName;

              models.erase(models.begin() + i);
              models.erase(models.begin() + j);
              models.push_back(mergedModel);

              i--;

              finished = false;
            }
          }
        }
      }
    }
  }

  //remove any single-grasp (unmerged) models
  for (int i = models.size() - 1; i >= 0; i --)
  {
    if (models[i].graspList.size() < 2)
    {
      models.erase(models.begin() + i);
      unusedModelIds.insert(unusedModelIds.begin(), i);
    }
  }
  ROS_INFO("Point cloud merging resulted in %lu models", models.size());

  for (unsigned int i = 0; i < models.size(); i++)
  {
    //add new models to database
    vector<graspdb::Grasp> grasps;
    grasps.resize(models[i].graspList.size());
    for (unsigned int j = 0; j < grasps.size(); j++)
    {
      grasps[j] = graspdb::Grasp(graspdb::Pose(models[i].graspList[j].grasp_pose), graspdb::Entity::UNSET_ID, models[i].graspList[j].eef_frame_id, models[i].graspList[j].successes, models[i].graspList[j].attempts);
    }
    PCLPointCloud2 tempCloud;
    sensor_msgs::PointCloud2 outCloud;
    toPCLPointCloud2(*models[i].pointCloud, tempCloud);
    pcl_conversions::fromPCL(tempCloud, outCloud);
    graspdb::GraspModel model(models[i].objectName, grasps, outCloud);
    if (graspdb->addGraspModel(model))
    {
      ROS_INFO("Added new model (number %d) to the database with id %d", i, model.getID());
    }
    else
    {
      ROS_INFO("Could not insert new model (number %d) into the database.", i);
    }
  }

  return models.size();
}

PointCloud<PointXYZRGB>::Ptr ModelGenerator::icpRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr,
    vector<rail_pick_and_place_msgs::GraspWithSuccessRate> baseGrasps,
    vector<rail_pick_and_place_msgs::GraspWithSuccessRate> targetGrasps,
    vector<rail_pick_and_place_msgs::GraspWithSuccessRate> *resultGrasps)
{
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

  //Transform grasps to the target cloud position and orientation
  Eigen::Matrix4f transform = icp.getFinalTransformation();

  tf::Matrix3x3 rotationMatrix(transform(0, 0), transform(0, 1), transform(0, 2),
      transform(1, 0), transform(1, 1), transform(1, 2),
      transform(2, 0), transform(2, 1), transform(2, 2));
  tf::Transform tfTransform;
  tf::Quaternion quat;
  rotationMatrix.getRotation(quat);
  tfTransform.setOrigin(tf::Vector3(transform(0, 3), transform(1, 3), transform(2, 3)));
  tfTransform.setRotation(quat);

  //TODO: do this without frames?
  ros::Time now = ros::Time::now();
  tfBroadcaster.sendTransform(tf::StampedTransform(tfTransform, now, "base_cloud_frame", "target_cloud_frame"));
  tfListener.waitForTransform("base_cloud_frame", "target_cloud_frame", now, ros::Duration(5.0));
  for (unsigned int i = 0; i < targetGrasps.size(); i++)
  {
    geometry_msgs::PoseStamped poseOut;
    geometry_msgs::PoseStamped tempPoseStamped;
    tempPoseStamped.pose = targetGrasps[i].grasp_pose.pose;
    tempPoseStamped.header.stamp = now;
    tempPoseStamped.header.frame_id = "target_cloud_frame";

    tfListener.transformPose("base_cloud_frame", tempPoseStamped, poseOut);
    targetGrasps[i].grasp_pose.pose = poseOut.pose;
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

  //classifyMerge(overlap, maxDstDiff, dstError, avgColorDiff);

  //return targetTransformedPtr;
  return resultPtr;
}

bool ModelGenerator::checkRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
{
  IterativeClosestPoint<PointXYZRGB, PointXYZRGB> icp;
  icp.setInputSource(targetCloudPtr);
  icp.setInputTarget(baseCloudPtr);
  PointCloud<PointXYZRGB>::Ptr targetTransformedPtr(new PointCloud<PointXYZRGB>);
  icp.align(*targetTransformedPtr);

  //debug:
  //float icpScore = icp.getFitnessScore();
  //ROS_INFO_STREAM("ICP convergence score: " << icpScore);

  float dstError = calculateRegistrationMetricDstError(baseCloudPtr, targetTransformedPtr);
  float overlap = calculateRegistrationMetricOverlap(baseCloudPtr, targetTransformedPtr, .005);
  float avgColorDiff = calculateRegistrationMetricColorRange(baseCloudPtr, targetTransformedPtr);
  float maxDstDiff = calculateRegistrationMetricDistance(baseCloudPtr, targetTransformedPtr);

  //debug:
  //ROS_INFO("Calculated distance error score: %f", dstError);
  //ROS_INFO("Calculated overlap score: %f", overlap);

  bool result = classifyMerge(overlap, maxDstDiff, dstError, avgColorDiff);

  return result;
}

float ModelGenerator::calculateRegistrationMetricDstError(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
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

float ModelGenerator::calculateRegistrationMetricOverlap(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, float dstThreshold)
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

  return score;
}

float ModelGenerator::calculateRegistrationMetricColorRange(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
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

float ModelGenerator::calculateRegistrationMetricDistance(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
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

void ModelGenerator::filterCloudOutliers(PointCloud<PointXYZRGB>::Ptr cloudPtr, double radius, int numNeighborThreshold)
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

void ModelGenerator::filterRedundentPoints(PointCloud<PointXYZRGB>::Ptr cloudPtr, double dstThreshold)
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

void ModelGenerator::translateToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, vector<rail_pick_and_place_msgs::GraspWithSuccessRate> *grasps)
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
  for (unsigned int i = 0; i < grasps->size(); i++)
  {
    grasps->at(i).grasp_pose.pose.position.x -= x;
    grasps->at(i).grasp_pose.pose.position.y -= y;
    grasps->at(i).grasp_pose.pose.position.z -= z;
  }
}

bool ModelGenerator::classifyMerge(float overlap, float maxDstDiff, float dstError, float avgColorDiff)
{
  if (overlap <= .795576)
  {
    if (maxDstDiff <= .002273)
    {
      if (dstError <= .053681)
        return false;
      else
        return true;
    }
    else
      return false;
  }
  else
  {
    if (avgColorDiff <= 91.010641)
    {
      if (maxDstDiff <= .000304)
        return false;
      else
        return true;
    }
    else
      return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_registration");

  ModelGenerator pcr;

  ros::spin();

  return 0;
}
