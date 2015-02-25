#include <rail_recognition/PCRegistration.h>

using namespace std;
using namespace pcl;
using namespace rail_recognition;

PCRegistration::PCRegistration()
{
  // private node handle
  ros::NodeHandle private_nh("~");

  //set output directory for generated models
  stringstream ss;
  ss << getenv("HOME");
  private_nh.param("output_dir", outputDirectory, ss.str());

  baseCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("pc_registration/base_cloud", 1);
  targetCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("pc_registration/target_cloud", 1);
  modelCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("pc_registration/model_cloud", 1);
  modelGraspsPublisher = n.advertise<geometry_msgs::PoseArray>("pc_registration/model_grasps", 1);

  readGraspClient = n.serviceClient<rail_recognition::ReadGrasp>("grasp_reader/read_grasps");
  generateModels = n.advertiseService("pc_registration/generate_models", &PCRegistration::generateModelsService, this);
  getModelNumbers = n.advertiseService("pc_registration/get_model_numbers", &PCRegistration::getModelNumbersService, this);
  displayModel = n.advertiseService("pc_registration/display_model", &PCRegistration::displayModelService, this);

  readGraspsAndModels();
}

void PCRegistration::readGraspsAndModels()
{
  ROS_INFO("Reading grasp demonstrations...");
  bool reading = true;
  stringstream ss;
  int count = 1;
  while (reading)
  {
    ss.str("");
    ss << "grasp_" << count << ".txt";
    Model readModel;
    reading = getModel(ss.str(), &readModel);
    if (reading)
    {
      individualGraspModels.push_back(readModel);
      count++;
    }
    else
      count--;
  }
  ROS_INFO("Read %d grasp demonstrations", count);

  ROS_INFO("Reading object models...");
  reading = true;
  count = 1;
  while (reading)
  {
    ss.str("");
    ss << "model_" << count << ".txt";
    Model readModel;
    reading = getModel(ss.str(), &readModel);
    if (reading)
    {
      mergedModels.push_back(readModel);
      count++;
    }
    else
      count--;
  }
  ROS_INFO("Read %d grasp demonstrations", count);
}

bool PCRegistration::getModel(string filename, Model *model)
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
    fromPCLPointCloud2(tempCloud, *model->pointCloud);
    baseCloudPublisher.publish(baseCloud);

    model->graspList = srv.response.grasp.gripperPoses;
    model->successesList = srv.response.grasp.successes;
    model->totalAttemptsList = srv.response.grasp.totalAttempts;
  }

  return srv.response.success;
}

bool PCRegistration::generateModelsService(rail_recognition::GenerateModels::Request &req, rail_recognition::GenerateModels::Response &res)
{
  vector<Model> models;
  models.resize(req.models.size());
  for (unsigned int i = 0; i < req.models.size(); i ++)
  {
    models[i].copy(individualGraspModels[req.models[i]]);
  }
  registerPointCloudsGraph(models, req.maxModelSize, res.unusedModelIds);
}

bool PCRegistration::getModelNumbersService(rail_recognition::GetModelNumbers::Request &req, rail_recognition::GetModelNumbers::Response &res)
{
  res.total_individual_grasps = individualGraspModels.size();
  res.total_merged_models = mergedModels.size();
  return true;
}

bool PCRegistration::displayModelService(rail_recognition::DisplayModel::Request &req, rail_recognition::DisplayModel::Response &res)
{
  sensor_msgs::PointCloud2 displayCloud;
  PCLPointCloud2 tempCloud;
  if (req.isMergedModel)
    toPCLPointCloud2(*mergedModels[req.modelId].pointCloud, tempCloud);
  else
    toPCLPointCloud2(*individualGraspModels[req.modelId].pointCloud, tempCloud);
  pcl_conversions::fromPCL(tempCloud, displayCloud);
  displayCloud.header.frame_id = req.frame;
  modelCloudPublisher.publish(displayCloud);

  geometry_msgs::PoseArray displayGrasps;
  displayGrasps.header = displayCloud.header;
  if (req.isMergedModel)
    displayGrasps.poses = mergedModels[req.modelId].graspList;
  else
    displayGrasps.poses = individualGraspModels[req.modelId].graspList;
  modelGraspsPublisher.publish(displayGrasps);
}

void PCRegistration::registerPointCloudsGraph(vector<Model> models, int maxModelSize, vector<int> unusedModelIds)
{
  PointCloud<PointXYZRGB>::Ptr baseCloudPtr(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr targetCloudPtr(new PointCloud<PointXYZRGB>);
  PointCloud<PointXYZRGB>::Ptr resultPtr(new PointCloud<PointXYZRGB>);

  vector<geometry_msgs::Pose> baseGraspList;
  vector<geometry_msgs::Pose> targetGraspList;
  vector<geometry_msgs::Pose> resultGraspList;

  //Filter point clouds to remove noise, translate them to the origin for easier visualization
  for (unsigned int i = 0; i < models.size(); i++)
  {
    cout << "to filter: point cloud of size: " << models[i].pointCloud->size() << endl;
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
              ROS_INFO("Skipping pair %d-%d as a merge would exceed the maximum model size.", i + 1, j + 1);
              continue;
            }
            else
            {
              ROS_INFO("Checking pair %d-%d", i + 1, j + 1);
            }

            baseCloudPtr = models[i].pointCloud;
            targetCloudPtr = models[j].pointCloud;

            if (checkRegistration(baseCloudPtr, targetCloudPtr))
            {
              ROS_INFO("Registration successful!");

              baseGraspList = models[i].graspList;
              targetGraspList = models[j].graspList;
              resultGraspList.clear();
              resultPtr = icpRegistration(baseCloudPtr, targetCloudPtr, baseGraspList, targetGraspList, &resultGraspList, false);
              filterRedundentPoints(resultPtr, DST_THRESHOLD);

              Model mergedModel;
              mergedModel.pointCloud = resultPtr;
              mergedModel.graspList = resultGraspList;

              vector<int> combinedSuccesses = models[i].successesList;
              vector<int> combinedTotalAttempts = models[i].totalAttemptsList;
              for (unsigned int k = 0; k < models[j].successesList.size(); k++)
              {
                combinedSuccesses.push_back(models[j].successesList[k]);
                combinedTotalAttempts.push_back(models[j].totalAttemptsList[k]);
              }
              mergedModel.successesList = combinedSuccesses;
              mergedModel.totalAttemptsList = combinedTotalAttempts;

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
    ROS_INFO("i: %d", i);
    if (models[i].graspList.size() < 2)
    {
      ROS_INFO("removing");
      models.erase(models.begin() + i);
      unusedModelIds.insert(unusedModelIds.begin(), i);
    }
  }
  ROS_INFO("Point cloud merging resulted in %lu models", models.size());

  for (unsigned int i = 0; i < models.size(); i++)
  {
    //save model

    sensor_msgs::PointCloud saveCloud;
    sensor_msgs::PointCloud2 tempSaveCloud;
    PCLPointCloud2 tempConvCloud;
    toPCLPointCloud2(*models[i].pointCloud, tempConvCloud);
    pcl_conversions::fromPCL(tempConvCloud, tempSaveCloud);
    sensor_msgs::convertPointCloud2ToPointCloud(tempSaveCloud, saveCloud);

    stringstream ss;
    ss << outputDirectory << "/model_" << mergedModels.size() + i + 1 << ".txt";

    ofstream myfile;
    myfile.open(ss.str().c_str());

    myfile << "reference_frame_id: base_footprint" << "\npointCloud:\n\theader:\n\t\tframe_id: base_footprint" << "\n\tpoints: [";
    for (unsigned int j = 0; j < saveCloud.points.size(); j++)
    {
      myfile << "[" << saveCloud.points[j].x << "," << saveCloud.points[j].y << "," << saveCloud.points[j].z << "]";
      if (j < saveCloud.points.size() - 1)
        myfile << ",";
    }
    myfile << "]\n\tchannels:";
    for (unsigned int j = 0; j < saveCloud.channels.size(); j++)
    {
      myfile << "\n\t\tname: " << saveCloud.channels[j].name << "\n\t\tvalues: [";
      for (unsigned int k = 0; k < saveCloud.channels[j].values.size(); k++)
      {
        uint32_t value = *(uint32_t *) (&saveCloud.channels[j].values[k]);
        myfile << value;
        if (k < saveCloud.channels[j].values.size() - 1)
          myfile << ",";
      }
      myfile << "]";
    }

    for (unsigned int j = 0; j < models[i].graspList.size(); j++)
    {
      ROS_INFO("Writing grasp %d", j);
      myfile << "\ngripperPose: " << "\n\tposition: [" << models[i].graspList[j].position.x << "," << models[i].graspList[j].position.y << "," << models[i].graspList[j].position.z << "]\n\torientation: [" << models[i].graspList[j].orientation.x << "," << models[i].graspList[j].orientation.y << "," << models[i].graspList[j].orientation.z << "," << models[i].graspList[j].orientation.w << "]";
    }

    myfile << "\nsuccesses: [";
    for (unsigned int j = 0; j < models[i].successesList.size(); j++)
    {
      myfile << models[i].successesList[j];
      if (j < models[i].successesList.size() - 1)
        myfile << ",";
    }
    myfile << "]";

    myfile << "\ntotalAttempts: [";
    for (unsigned int j = 0; j < models[i].totalAttemptsList.size(); j++)
    {
      myfile << models[i].totalAttemptsList[j];
      if (j < models[i].totalAttemptsList.size() - 1)
        myfile << ",";
    }
    myfile << "]";

    myfile.close();

    ROS_INFO("Finished writing file model_%d", i);
  }

  for (unsigned int i = 0; i < models.size(); i ++)
  {
    mergedModels.push_back(models[i]);
  }

  ROS_INFO("---------------------------");
  ROS_INFO("Models saved to %s, move them to the models directory of the rail_recognition package to use them for recognition.", outputDirectory.c_str());
}

PointCloud<PointXYZRGB>::Ptr PCRegistration::icpRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, vector<geometry_msgs::Pose> baseGrasps, vector<geometry_msgs::Pose> targetGrasps, vector<geometry_msgs::Pose> *resultGrasps)
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

  ros::Time now = ros::Time::now();
  tfBroadcaster.sendTransform(tf::StampedTransform(tfTransform, now, "base_cloud_frame", "target_cloud_frame"));
  tfListener.waitForTransform("base_cloud_frame", "target_cloud_frame", now, ros::Duration(5.0));
  for (unsigned int i = 0; i < targetGrasps.size(); i++)
  {
    geometry_msgs::PoseStamped poseOut;
    geometry_msgs::PoseStamped tempPoseStamped;
    tempPoseStamped.pose = targetGrasps[i];
    tempPoseStamped.header.stamp = now;
    tempPoseStamped.header.frame_id = "target_cloud_frame";

    tfListener.transformPose("base_cloud_frame", tempPoseStamped, poseOut);
    targetGrasps[i] = poseOut.pose;
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

bool PCRegistration::checkRegistration(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
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

float PCRegistration::calculateRegistrationMetricDstError(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
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

float PCRegistration::calculateRegistrationMetricOverlap(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr, float dstThreshold)
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

float PCRegistration::calculateRegistrationMetricColorRange(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
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

float PCRegistration::calculateRegistrationMetricDistance(PointCloud<PointXYZRGB>::Ptr baseCloudPtr, PointCloud<PointXYZRGB>::Ptr targetCloudPtr)
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

void PCRegistration::filterCloudOutliers(PointCloud<PointXYZRGB>::Ptr cloudPtr, double radius, int numNeighborThreshold)
{
  ROS_INFO("1");
  KdTreeFLANN<PointXYZRGB> searchTree(new KdTreeFLANN<PointXYZRGB>);
  ROS_INFO("2");
  searchTree.setInputCloud(cloudPtr);
  ROS_INFO("3");
  vector<int> removeIndices;
  vector<int> indices;
  vector<float> distances;
  ROS_INFO("4");

  for (unsigned int i = 0; i < cloudPtr->size(); i++)
  {
    int neighbors = searchTree.radiusSearch(cloudPtr->at(i), radius, indices, distances);
    if (neighbors < numNeighborThreshold)
      removeIndices.push_back(i);
  }
  ROS_INFO("5");

  sort(removeIndices.begin(), removeIndices.end());
  reverse(removeIndices.begin(), removeIndices.end());

  ROS_INFO("Found %lu points to filter", removeIndices.size());

  for (int i = (int) (removeIndices.size()) - 1; i >= 0; i--)
  {
    cloudPtr->erase(cloudPtr->begin() + i);
  }
}

void PCRegistration::filterRedundentPoints(PointCloud<PointXYZRGB>::Ptr cloudPtr, double dstThreshold)
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

void PCRegistration::translateToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, vector<geometry_msgs::Pose> *grasps)
{
  float x = 0;
  float y = 0;
  float z = 0;

  for (unsigned int i = 0; i < cloudPtr->size(); i++)
  {
    x += cloudPtr->at(i).x;
    y += cloudPtr->at(i).y;
    z += cloudPtr->at(i).z;
  }
  x /= cloudPtr->size();
  y /= cloudPtr->size();
  z /= cloudPtr->size();

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
    (*grasps)[i].position.x -= x;
    (*grasps)[i].position.y -= y;
    (*grasps)[i].position.z -= z;
  }
}

void PCRegistration::publishTest()
{
  baseCloudPublisher.publish(baseCloud);
  targetCloudPublisher.publish(targetCloud);
}

bool PCRegistration::classifyMerge(float overlap, float maxDstDiff, float dstError, float avgColorDiff)
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

  PCRegistration pcr;

  //pcr.pairwiseRegisterPointclouds();
  //pcr.registerPointcloudsGraph();

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    pcr.publishTest();
    loop_rate.sleep();
  }

  return 0;
}
