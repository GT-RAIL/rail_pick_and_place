// RAIL Recognition
#include "rail_recognition/PointCloudMetrics.h"
#include "rail_recognition/PointCloudRecognizer.h"

// PCL
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

using namespace std;
using namespace rail::pick_and_place;

PointCloudRecognizer::PointCloudRecognizer()
{
}

bool PointCloudRecognizer::recognizeObject(rail_manipulation_msgs::SegmentedObject &object,
    const vector<graspdb::GraspModel> &candidates) const
{
  // make sure we have some candidates
  if (candidates.empty())
  {
    ROS_WARN("Candidate object list is empty. Nothing to compare segmented object to.");
    return false;
  }
  if (object.point_cloud.data.empty())
  {
    ROS_WARN("Segmented object point cloud is empty. Nothing to compare candidate objects to.");
    return false;
  }

  // convert to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  point_cloud_metrics::rosPointCloud2ToPCLPointCloud(object.point_cloud, object_point_cloud);

  // pre-process input cloud
  point_cloud_metrics::filterPointCloudOutliers(object_point_cloud);
  point_cloud_metrics::transformToOrigin(object_point_cloud, object.centroid);

  // perform recognition
  double min_score = numeric_limits<double>::infinity();
  size_t min_index;
  tf2::Transform min_icp_tf;
  bool min_swapped;
  for (size_t i = 0; i < candidates.size(); i++)
  {
    // quick check for a valid point cloud
    if (candidates[i].getPointCloud().data.size() > 0)
    {
      // convert the candidate point cloud to a PCL point cloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr candidate_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      point_cloud_metrics::rosPointCloud2ToPCLPointCloud(candidates[i].getPointCloud(), candidate_point_cloud);

      tf2::Transform cur_icp_tf;
      bool cur_swapped;
      double score = this->scoreRegistration(candidate_point_cloud, object_point_cloud, cur_icp_tf, cur_swapped);
      if (score < min_score)
      {
        min_score = score;
        min_index = i;
        min_icp_tf = cur_icp_tf;
        min_swapped = cur_swapped;
      }
    }
  }

  // check if there is enough confidence
  if (min_score > SCORE_CONFIDENCE_THRESHOLD)
  {
    return false;
  }

  // fill in recognition information
  object.name = candidates[min_index].getObjectName();
  object.model_id = candidates[min_index].getID();
  object.confidence = min_score;
  object.recognized = true;
  // TODO infer object orientation
  object.orientation.w = 1.0;
  object.grasps.clear();

  // extract possible grasps for this model
  vector<graspdb::Grasp> possible_grasps;
  this->computeGraspList(min_icp_tf, min_swapped, object.centroid, candidates[min_index].getGrasps(), possible_grasps);

  // sort and remove any grasps with 0 success rates
  vector<double> success_rates;
  for (size_t i = 0; i < possible_grasps.size(); i++)
  {
    double rate = possible_grasps[i].getSuccessRate();
    geometry_msgs::PoseStamped pose = possible_grasps[i].getGraspPose().toROSPoseStampedMessage();
    // fix the frame ID
    pose.header.frame_id = object.point_cloud.header.frame_id;

    // check the success rate -- any non-zero or non-attempted grasp
    if (rate > 0 || possible_grasps[i].getAttempts() == 0)
    {
      // place it in order
      bool inserted = false;
      for (size_t j = 0; j < success_rates.size(); j++)
      {
        if (rate <= success_rates[j])
        {
          object.grasps.insert(object.grasps.begin() + j, pose);
          success_rates.insert(success_rates.begin() + j, rate);
          inserted = true;
          break;
        }
      }

      if (!inserted)
      {
        object.grasps.push_back(pose);
        success_rates.push_back(rate);
      }
    }
  }

  return true;
}

double PointCloudRecognizer::scoreRegistration(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr candidate,
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr object, tf2::Transform &tf_icp, bool &icp_swapped) const
{
  // use the larger as the base cloud
  icp_swapped = object->size() > candidate->size();
  // use ICP to for matching
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (icp_swapped)
  {
    tf_icp = point_cloud_metrics::performICP(object, candidate, aligned);
  } else
  {
    tf_icp = point_cloud_metrics::performICP(candidate, object, aligned);
  }

  // calculate the distance and color error
  double distance_error = point_cloud_metrics::calculateRegistrationMetricDistanceError(candidate, aligned);
  double color_error = point_cloud_metrics::calculateRegistrationMetricOverlap(candidate, aligned, true);

  // calculate the final weighted result
  double result = ALPHA * (3.0 * distance_error) + (1.0 - ALPHA) * (color_error / 100.0);
  return result;
}

void PointCloudRecognizer::computeGraspList(const tf2::Transform &tf_icp, const bool icp_swapped,
    const geometry_msgs::Point &centroid, const vector<graspdb::Grasp> &candidate_grasps,
    vector<graspdb::Grasp> &grasps) const
{
  // ensure an empty list
  grasps.clear();

  // transform each pose
  for (size_t i = 0; i < candidate_grasps.size(); i++)
  {
    // convert to tf2 matrix
    tf2::Transform tf_pose = candidate_grasps[i].getGraspPose().toTF2Transform();

    // push back the basic information
    grasps.push_back(candidate_grasps[i]);

    tf2::Transform result;
    if (icp_swapped)
    {
      result = tf_icp * tf_pose;
    } else
    {
      // use the inverse for the result
      result = tf_icp.inverseTimes(tf_pose);
    }

    // correct for the origin transform
    result.getOrigin().setX(result.getOrigin().getX() + centroid.x);
    result.getOrigin().setY(result.getOrigin().getY() + centroid.y);
    result.getOrigin().setZ(result.getOrigin().getZ() + centroid.z);

    // copy over the values
    grasps[i].setGraspPose(graspdb::Pose(grasps[i].getGraspPose().getRobotFixedFrameID(), result));
  }
}
