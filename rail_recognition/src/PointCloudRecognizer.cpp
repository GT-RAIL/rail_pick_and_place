#include <rail_recognition/PointCloudRecognizer.h>
#include <rail_recognition/PointCloudMetrics.h>

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
  Eigen::Matrix4f min_icp_tf;
  bool min_swapped;
  for (size_t i = 0; i < candidates.size(); i++)
  {
    // convert the candidate point cloud to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr candidate_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    point_cloud_metrics::rosPointCloud2ToPCLPointCloud(candidates[i].getPointCloud(), candidate_point_cloud);

    Eigen::Matrix4f cur_icp_tf;
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
  vector<graspdb::Grasp> possible_grasps = this->computeGraspList(min_icp_tf, min_swapped, object.centroid,
      candidates[min_index].getGrasps());

  // sort and remove any grasps with 0 success rates
  vector<double> success_rates;
  for (size_t i = 0; i < possible_grasps.size(); i++)
  {
    double rate = possible_grasps[i].getSuccessRate();

    // check the success rate
    if (rate > 0)
    {
      // place it in order
      bool inserted = false;
      for (size_t j = 0; j < success_rates.size(); j++)
      {
        if (rate <= success_rates[j])
        {
          object.grasps.insert(object.grasps.begin() + j, possible_grasps[i].getGraspPose().toROSPoseStampedMessage());
          success_rates.insert(success_rates.begin() + j, rate);
          inserted = true;
          break;
        }
      }

      if (!inserted)
      {
        object.grasps.push_back(possible_grasps[i].getGraspPose().toROSPoseStampedMessage());
        success_rates.push_back(rate);
      }
    }
  }

  return true;
}

double PointCloudRecognizer::scoreRegistration(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr candidate,
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr object, Eigen::Matrix4f &icp_tf, bool &icp_swapped) const
{
  // use the larger as the base cloud
  icp_swapped = object->size() > candidate->size();
  // use ICP to for matching
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  if (icp_swapped)
  {
    icp.setInputTarget(object);
    icp.setInputSource(candidate);
  } else
  {
    icp.setInputTarget(candidate);
    icp.setInputSource(object);
  }

  // run ICP on the two point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGB>);
  icp.align(*aligned);
  // extract the final transform
  icp_tf = icp.getFinalTransformation();

  // calculate the distance and color error
  double distance_error = point_cloud_metrics::calculateRegistrationMetricDistanceError(candidate, aligned);
  double color_error = point_cloud_metrics::calculateRegistrationMetricOverlap(candidate, aligned, true);

  // calculate the final weighted result
  double result = ALPHA * (3.0 * distance_error) + (1.0 - ALPHA) * (color_error / 100.0);
  return result;
}


vector<graspdb::Grasp> PointCloudRecognizer::computeGraspList(const Eigen::Matrix4f &icp_transform,
    const bool icp_swapped, const geometry_msgs::Point &centroid, const vector<graspdb::Grasp> &candidate_grasps) const
{
  vector<graspdb::Grasp> grasps;

  // convert to a TF2 matrix
  tf2::Matrix3x3 rotation(icp_transform(0, 0), icp_transform(0, 1), icp_transform(0, 2),
      icp_transform(1, 0), icp_transform(1, 1), icp_transform(1, 2),
      icp_transform(2, 0), icp_transform(2, 1), icp_transform(2, 2));
  tf2::Vector3 translation(icp_transform(0, 3), icp_transform(1, 3), icp_transform(2, 3));
  tf2::Transform tf_icp(rotation, translation);

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
      result.mult(tf_icp, tf_pose);
    } else
    {
      // use the inverse for the result
      tf2::Transform inverse = tf_icp.inverse();
      result.mult(inverse, tf_pose);
    }

    // correct for the origin transform
    result.getOrigin().setX(result.getOrigin().getX() + centroid.x);
    result.getOrigin().setY(result.getOrigin().getY() + centroid.y);
    result.getOrigin().setZ(result.getOrigin().getZ() + centroid.z);

    // copy over the values
    grasps[i].setGraspPose(graspdb::Pose(grasps[i].getGraspPose().getRobotFixedFrameID(), result));
  }

  return grasps;
}
