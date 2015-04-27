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
#include <rail_recognition/PointCloudMetrics.h>

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
  remove_object_srv_ = private_node_.advertiseService("remove_object",
                                                      &ObjectRecognitionListener::removeObjectCallback, this);

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
  //lock for the object list
  boost::mutex::scoped_lock lock(mutex_);

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
  bool something_combined = false;
  for (size_t i = 0; i < object_list_.objects.size() - 1; i++)
  {
    for (size_t j = i + 1; j < object_list_.objects.size(); j++)
    {
      // two models can potentially be combined if they are the same object type
      if (object_list_.objects[i].recognized && object_list_.objects[j].recognized
          && object_list_.objects[i].name == object_list_.objects[j].name)
      {
        double distance = sqrt(pow(object_list_.objects[i].center.x - object_list_.objects[j].center.x, 2)
                               + pow(object_list_.objects[i].center.y - object_list_.objects[j].center.y, 2)
                               + pow(object_list_.objects[i].center.z - object_list_.objects[j].center.z, 2));
        if (distance <= SAME_OBJECT_DIST_THRESHOLD)
        {
          rail_manipulation_msgs::SegmentedObject combined;
          this->combineModels(object_list_.objects[i], object_list_.objects[j], combined);
          object_list_.objects[i] = combined;
          object_list_.objects.erase(object_list_.objects.begin() + j);
          j--;
          something_combined = true;
        }
      }
    }
  }

  if (something_combined)
  {
    // re-index marker ids since the object_list_.objects indexing has changed
    for (size_t i = 0; i < object_list_.objects.size(); i++)
    {
      object_list_.objects[i].marker.id = i;
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

bool ObjectRecognitionListener::removeObjectCallback(rail_pick_and_place_msgs::RemoveObject::Request &req,
    rail_pick_and_place_msgs::RemoveObject::Response &res)
{
  //lock for the object list
  boost::mutex::scoped_lock lock(mutex_);

  if (req.index < object_list_.objects.size())
  {
    // remove
    object_list_.objects.erase(object_list_.objects.begin() + req.index);
    // set header information
    object_list_.header.seq++;
    object_list_.header.stamp = ros::Time::now();
    object_list_.cleared = false;
    // republish
    recognized_objects_pub_.publish(object_list_);
    return true;
  } else
  {
    ROS_ERROR("Attempted to remove index %d from list of size %ld.", req.index, object_list_.objects.size());
    return false;
  }
}

bool ObjectRecognitionListener::comparePointClouds(const sensor_msgs::PointCloud2 &pc1,
    const sensor_msgs::PointCloud2 &pc2) const
{
  // check the size first then compare
  return (pc1.data.size() == pc2.data.size()) && (pc1.data == pc1.data);
}

void ObjectRecognitionListener::combineModels(const rail_manipulation_msgs::SegmentedObject &model1,
    const rail_manipulation_msgs::SegmentedObject &model2, rail_manipulation_msgs::SegmentedObject &combined) const
{
  ROS_INFO("Combining two %s models...", model1.name.c_str());

  // set members that won't change
  combined.name = model1.name;
  combined.recognized = model1.recognized;
  // keep the first model ID, as merging them won't make sense
  combined.model_id = model1.model_id;
  // TODO: calculate the merged image, for now it uses just the largest
  combined.image = (model1.image.data.size() > model2.image.data.size()) ? model1.image : model2.image;
  combined.confidence = max(model1.confidence, model2.confidence);

  // combine point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  point_cloud_metrics::rosPointCloud2ToPCLPointCloud(model1.point_cloud, cloud1);
  point_cloud_metrics::rosPointCloud2ToPCLPointCloud(model2.point_cloud, cloud2);
  *combined_cloud = *cloud1 + *cloud2;
  point_cloud_metrics::pclPointCloudToROSPointCloud2(combined_cloud, combined.point_cloud);

  // calculate new point cloud attributes (center, centroid, depth, height, width, orientation)
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*combined_cloud, centroid);
  combined.centroid.x = centroid[0];
  combined.centroid.y = centroid[1];
  combined.centroid.z = centroid[2];

  // calculate the new bounding box
  int x_idx, y_idx, z_idx;
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*combined_cloud, min_pt, max_pt);
  combined.width = max_pt[0] - min_pt[0];
  combined.depth = max_pt[1] - min_pt[1];
  combined.height = max_pt[2] - min_pt[2];

  // calculate the new center
  combined.center.x = (max_pt[0] + min_pt[0]) / 2.0;
  combined.center.y = (max_pt[1] + min_pt[1]) / 2.0;
  combined.center.z = (max_pt[2] + min_pt[2]) / 2.0;

  // TODO: calculate combined orientation here once orientation is implemented for segmented objects
  combined.orientation = model1.orientation;

  // combine the two markers
  combined.marker = model1.marker;
  combined.marker.points.insert(combined.marker.points.end(), model2.marker.points.begin(), model2.marker.points.end());

  // set average RGB
  combined.marker.color.r = (model1.marker.color.r + model2.marker.color.r) / 2.0;
  combined.marker.color.g = (model1.marker.color.g + model2.marker.color.g) / 2.0;
  combined.marker.color.b = (model1.marker.color.b + model2.marker.color.b) / 2.0;

  // combine grasp lists and maintain order
  combined.grasps = model1.grasps;
  for (size_t i = 0; i < model2.grasps.size(); i++)
  {
    //add un-attempted grasps to the front of the list
    if (model2.grasps[i].attempts == 0)
    {
      combined.grasps.insert(combined.grasps.begin(), model2.grasps[i]);
    }
    else
    {
      // sort by success rate
      bool inserted = false;
      double success_rate = ((double) model2.grasps[i].successes) / ((double) model2.grasps[i].attempts);
      for (size_t j = 0; j < combined.grasps.size(); j++)
      {
        double compare_rate;
        if (combined.grasps[j].attempts == 0)
        {
          compare_rate = 1.0;
        } else
        {
          compare_rate = ((double) combined.grasps[j].successes) / ((double) combined.grasps[j].attempts);
        }

        if (success_rate >= compare_rate)
        {
          combined.grasps.insert(combined.grasps.begin() + j, model2.grasps[i]);
          inserted = true;
          break;
        }
      }

      // add to the end
      if (!inserted)
      {
        combined.grasps.push_back(model2.grasps[i]);
      }
    }
  }
}
