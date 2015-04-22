/*!
 * \file ObjectRecognitionListener.cpp
 * \brief The object recognition listener node object.
 *
 * The object recognition listener will listen to a specified SegmentedObjectsArray topic and attempt to recognize
 * all segmented objects. The new list are republished on a separate topic.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

// RAIL Recognition
#include "rail_recognition/ObjectRecognitionListener.h"
#include "rail_recognition/PointCloudRecognizer.h"

// ROS
#include <geometry_msgs/PoseArray.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace rail::pick_and_place;

ObjectRecognitionListener::ObjectRecognitionListener() : private_node_("~")
{
  // set defaults
  debug_ = DEFAULT_DEBUG;
  string segmented_objects_topic("/segmentation/segmented_objects");
  int port = graspdb::Client::DEFAULT_PORT;
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");

  // grab any parameters we need
  private_node_.getParam("debug", debug_);
  private_node_.getParam("segmented_objects_topic", segmented_objects_topic);
  node_.getParam("/graspdb/host", host);
  node_.getParam("/graspdb/port", port);
  node_.getParam("/graspdb/user", user);
  node_.getParam("/graspdb/password", password);
  node_.getParam("/graspdb/db", db);

  // connect to the grasp database
  graspdb_ = new graspdb::Client(host, port, user, password, db);
  okay_ = graspdb_->connect();

  // setup a debug publisher if we need it
  if (debug_)
  {
    debug_pub_ = private_node_.advertise<geometry_msgs::PoseArray>("debug", 1, true);
  }

  segmented_objects_sub_ = node_.subscribe(segmented_objects_topic, 1,
      &ObjectRecognitionListener::segmentedObjectsCallback, this);
  recognized_objects_pub_ = private_node_.advertise<rail_manipulation_msgs::SegmentedObjectList>(
      "recognized_objects", 1);

  if (okay_)
  {
    ROS_INFO("Object Recognition Listener Successfully Initialized");
  }
}

ObjectRecognitionListener::~ObjectRecognitionListener()
{
  // cleanup
  graspdb_->disconnect();
  delete graspdb_;
}

bool ObjectRecognitionListener::okay() const
{
  return okay_;
}

void ObjectRecognitionListener::segmentedObjectsCallback(
    const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &objects)
{
  ROS_INFO("Received %li segmented objects.", objects->objects.size());

  // check against the old list to prevent throwing out data
  vector<rail_manipulation_msgs::SegmentedObject> new_list;
  for (size_t i = 0; i < objects->objects.size(); i++)
  {
    bool matched = false;
    // search for a match on the point cloud
    for (size_t j = 0; j < object_list_.objects.size(); j++)
    {
      // only do a compare if we previously recognized the object
      if (object_list_.objects[j].recognized &&
          this->comparePointClouds(objects->objects[i].point_cloud, object_list_.objects[j].point_cloud))
      {
        ROS_INFO("Found a match from previously recognized objects.");
        matched = true;
        new_list.push_back(object_list_.objects[j]);
        break;
      }
    }

    // check if we didn't match
    if (!matched)
    {
      new_list.push_back(objects->objects[i]);
    }
  }

  // store the list
  object_list_.objects = new_list;

  // run recognition
  ROS_INFO("Running recognition...");
  // populate candidates
  vector<graspdb::GraspModel> candidates;
  graspdb_->loadGraspModels(candidates);
  // convert to PCL grasp models
  vector<PCLGraspModel> pcl_candidates;
  for (size_t i = 0; i < candidates.size(); i++)
  {
    pcl_candidates.push_back(PCLGraspModel(candidates[i]));
  }

  // go through the current list
  PointCloudRecognizer recognizer;
  for (size_t i = 0; i < object_list_.objects.size(); i++)
  {
    // check if it is already recognized
    rail_manipulation_msgs::SegmentedObject &object = object_list_.objects[i];
    if (!object.recognized)
    {
      // perform recognition
      recognizer.recognizeObject(object, pcl_candidates);
    }
  }

  // check if any recognized models should be combined
  if (object_list_.objects.size() > 1)
  {
    bool something_combined = false;

    for (size_t i = 0; i < object_list_.objects.size() - 1; i ++)
    {
      for (size_t j = i + 1; j < object_list_.objects.size(); j ++)
      {
        // two models can potentially be combined if they are the same object type
        if (object_list_.objects[i].recognized && object_list_.objects[j].recognized
              && object_list_.objects[i].name == object_list_.objects[j].name)
        {
          double dst = sqrt(pow(object_list_.objects[i].center.x - object_list_.objects[j].center.x, 2)
                  + pow(object_list_.objects[i].center.y - object_list_.objects[j].center.y, 2)
                  + pow(object_list_.objects[i].center.z - object_list_.objects[j].center.z, 2));
          if (dst <= SAME_OBJECT_DST_THRESHOLD)
          {
            object_list_.objects[i] = combineModels(object_list_.objects[i], object_list_.objects[j]);
            object_list_.objects.erase(object_list_.objects.begin() + j);
            j --;
            something_combined = true;
          }
        }
      }
    }

    if (something_combined)
    {
      //re-index marker ids since the object_list_.objects indexing has changed
      for (size_t i = 0; i < object_list_.objects.size(); i ++)
      {
        object_list_.objects[i].marker.id = i;
      }
    }
  }

  // republish the new list
  recognized_objects_pub_.publish(object_list_);
  // check for debug publishing
  if (debug_)
  {
    geometry_msgs::PoseArray poses;
    for (size_t i = 0; i < object_list_.objects.size(); i++)
    {
      for (size_t j = 0; j < object_list_.objects[i].grasps.size(); j++)
      {
        poses.header = object_list_.objects[i].grasps[j].grasp_pose.header;
        poses.poses.push_back(object_list_.objects[i].grasps[j].grasp_pose.pose);
      }
    }
    debug_pub_.publish(poses);
  }

  ROS_INFO("New recognized objects published.");
}

bool ObjectRecognitionListener::comparePointClouds(const sensor_msgs::PointCloud2 &pc1,
    const sensor_msgs::PointCloud2 &pc2) const
{
  // check the size first then compare
  return (pc1.data.size() == pc2.data.size()) && (pc1.data == pc1.data);
}

rail_manipulation_msgs::SegmentedObject ObjectRecognitionListener::combineModels(
  const rail_manipulation_msgs::SegmentedObject model1, const rail_manipulation_msgs::SegmentedObject model2)
{
  ROS_INFO("Combining two %s models...", model1.name.c_str());

  rail_manipulation_msgs::SegmentedObject combined_model;

  // set members that won't change
  combined_model.name = model1.name;
  combined_model.recognized = model1.recognized;
  combined_model.model_id = model1.model_id;  // keep the first model id, as merging them won't make sense
  combined_model.image = model1.image;  // TODO: calculate the merged image, for now it uses just the first model's
  combined_model.confidence = model1.confidence;  // TODO: figure out how to handle combining models of different confidence

  ROS_INFO("Combing point clouds...");

  // combine point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  point_cloud_metrics::rosPointCloud2ToPCLPointCloud(model1.point_cloud, cloud1);
  point_cloud_metrics::rosPointCloud2ToPCLPointCloud(model2.point_cloud, cloud2);
  *combined_cloud = *cloud1 + *cloud2;
  point_cloud_metrics::pclPointCloudToROSPointCloud2(cloud1, combined_model.point_cloud);

  ROS_INFO("Computing new centroid and bounding box information...");

  // calculate new point cloud attributes (center, centroid, depth, height, width, orientation)
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*combined_cloud, centroid);
  combined_model.centroid.x = centroid[0];
  combined_model.centroid.y = centroid[1];
  combined_model.centroid.z = centroid[2];

  // calculate the new bounding box
  int x_idx, y_idx, z_idx;
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*combined_cloud, min_pt, max_pt);
  combined_model.width = max_pt[0] - min_pt[0];
  combined_model.depth = max_pt[1] - min_pt[1];
  combined_model.height = max_pt[2] - min_pt[2];

  // calculate the new center
  combined_model.center.x = .5*(max_pt[0] + min_pt[0]);
  combined_model.center.y = .5*(max_pt[1] + min_pt[1]);
  combined_model.center.z = .5*(max_pt[2] + min_pt[2]);

  // TODO: Calculate combined orientation here once orientation is implemented for segmented objects...

  ROS_INFO("Creating new visualization marker...");

  // combine markers
  combined_model.marker = model1.marker;

  pcl::PCLPointCloud2 downsampled;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
  pcl::PCLPointCloud2::Ptr converted(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*combined_cloud, *converted);
  voxel_grid.setInputCloud(converted);
  voxel_grid.setLeafSize(DOWNSAMPLE_LEAF_SIZE, DOWNSAMPLE_LEAF_SIZE, DOWNSAMPLE_LEAF_SIZE);
  voxel_grid.filter(downsampled);

  // convert to an easy to use point cloud message
  sensor_msgs::PointCloud2 pc2_msg;
  pcl_conversions::fromPCL(downsampled, pc2_msg);
  sensor_msgs::PointCloud pc_msg;
  sensor_msgs::convertPointCloud2ToPointCloud(pc2_msg, pc_msg);

  // place in the marker message
  combined_model.marker.points.resize(pc_msg.points.size());
  int r = 0, g = 0, b = 0;
  for (size_t j = 0; j < pc_msg.points.size(); j++)
  {
    combined_model.marker.points[j].x = pc_msg.points[j].x;
    combined_model.marker.points[j].y = pc_msg.points[j].y;
    combined_model.marker.points[j].z = pc_msg.points[j].z;

    // use average RGB
    uint32_t rgb = *reinterpret_cast<int *>(&pc_msg.channels[0].values[j]);
    r += (int) ((rgb >> 16) & 0x0000ff);
    g += (int) ((rgb >> 8) & 0x0000ff);
    b += (int) ((rgb) & 0x0000ff);
  }

  // set average RGB
  combined_model.marker.color.r = ((float) r / (float) pc_msg.points.size()) / 255.0;
  combined_model.marker.color.g = ((float) g / (float) pc_msg.points.size()) / 255.0;
  combined_model.marker.color.b = ((float) b / (float) pc_msg.points.size()) / 255.0;
  combined_model.marker.color.a = 1.0;

  ROS_INFO("Combing grasps...");

  // combine grasp lists
  combined_model.grasps = model1.grasps;
  for (size_t i = 0; i < model2.grasps.size(); i ++)
  {
    //add unattempted grasps to the front of the list
    if (model2.grasps[i].attempts == 0)
    {
      combined_model.grasps.insert(combined_model.grasps.begin(), model2.grasps[i]);
    }
    else
    {
      bool inserted = false;
      double success_rate = model2.grasps[i].successes / model2.grasps[i].attempts;
      for (size_t j = 0; j < combined_model.grasps.size(); j ++)
      {
        double compare_rate;
        if (combined_model.grasps[j].attempts == 0)
          compare_rate = 1.0;
        else
          compare_rate = combined_model.grasps[j].successes / combined_model.grasps[j].attempts;

        if (success_rate >= compare_rate)
        {
          combined_model.grasps.insert(combined_model.grasps.begin() + j, model2.grasps[i]);
          inserted = true;
          break;
        }
      }
      if (!inserted)
        combined_model.grasps.push_back(model2.grasps[i]);
    }
  }

  ROS_INFO("Finished combing %s models.", combined_model.name.c_str());

  return combined_model;
}
