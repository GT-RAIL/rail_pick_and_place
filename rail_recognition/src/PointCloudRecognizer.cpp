#include <rail_recognition/PointCloudRecognizer.h>

// ROS
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>

using namespace std;
using namespace pcl;
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

  // convert to a PCL point cloud
  pcl::PCLPointCloud2 object_converter;
  pcl_conversions::toPCL(object.point_cloud, object_converter);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(object_converter, *object_point_cloud);

  // pre-process input cloud
  this->filterPointCloudOutliers(object_point_cloud);
  this->translateToOrigin(object_point_cloud, object.centroid);

  // perform recognition
  double min_score = numeric_limits<double>::infinity();
  size_t min_index;
  Eigen::Matrix4f min_icp_tf;
  bool min_swapped;
  for (size_t i = 0; i < candidates.size(); i++)
  {
    // convert the candidate point cloud to a PCL point cloud
    pcl::PCLPointCloud2 cur_converter;
    pcl_conversions::toPCL(candidates[i].getPointCloud(), cur_converter);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr candidate_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cur_converter, *candidate_point_cloud);

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
  object.recognized = true;
  object.grasps.clear();

  // extract possible grasps for this model
  vector<rail_pick_and_place_msgs::GraspWithSuccessRate> possible_grasps;
  this->transformGrasps(min_icp_tf, min_swapped, object.centroid,
      candidates[min_index].toROSGraspModelMessage().grasps, possible_grasps);

  // sort and remove any grasps with 0 success rates
  vector<double> success_rates;
  for (size_t i = 0; i < possible_grasps.size(); i++)
  {
    double rate = (possible_grasps[i].attempts > 0) ?
        ((double) possible_grasps[i].successes) / ((double) possible_grasps[i].attempts) : 0.0;

    // check the success rate
    if (rate > 0)
    {
      // place it in order
      bool inserted = false;
      for (size_t j = 0; j < success_rates.size(); j++)
      {
        if (rate <= success_rates[j])
        {
          object.grasps.insert(object.grasps.begin() + j, possible_grasps[i].grasp_pose);
          success_rates.insert(success_rates.begin() + j, rate);
          inserted = true;
          break;
        }
      }

      if (!inserted)
      {
        object.grasps.push_back(possible_grasps[i].grasp_pose);
        success_rates.push_back(rate);
      }
    }
  }

  return true;
}

void PointCloudRecognizer::filterPointCloudOutliers(PointCloud<PointXYZRGB>::Ptr pc) const
{
  // use a KD tree to search
  pcl::KdTreeFLANN<pcl::PointXYZRGB> search_tree;
  search_tree.setInputCloud(pc);

  // check each point
  pcl::IndicesPtr to_keep(new vector<int>);
  for (size_t i = 0; i < pc->size(); i++)
  {
    vector<int> indices;
    vector<float> distances;
    // check how many neighbors pass the test
    int neighbors = search_tree.radiusSearch(pc->at(i), FILTER_SEARCH_RADIUS, indices, distances);
    if (neighbors >= FILTER_MIN_NUM_NEIGHBORS)
    {
      to_keep->push_back(i);
    }
  }

  // extract the point we wish to keep
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(pc);
  extract.setIndices(to_keep);
  extract.filter(*pc);
}

void PointCloudRecognizer::translateToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc,
    const geometry_msgs::Point &centroid) const
{
  // transformation matrix
  Eigen::Matrix4f tf;
  tf << 1, 0, 0, -centroid.x,
      0, 1, 0, -centroid.y,
      0, 0, 1, -centroid.z,
      0, 0, 0, 1;

  // transform the point cloud
  pcl::transformPointCloud(*pc, *pc, tf);
}

double PointCloudRecognizer::scoreRegistration(PointCloud<PointXYZRGB>::ConstPtr candidate,
    PointCloud<PointXYZRGB>::ConstPtr object, Eigen::Matrix4f &icp_tf, bool &icp_swapped) const
{
  // use the larger as the base cloud
  icp_swapped = object->size() > candidate->size();
  // use ICP to for matching
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  if (icp_swapped)
  {
    icp.setInputSource(candidate);
    icp.setInputTarget(object);
  } else
  {
    icp.setInputSource(object);
    icp.setInputTarget(candidate);
  }

  // run ICP on the two point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZRGB>);
  icp.align(*aligned);
  // extract the final transform
  icp_tf = icp.getFinalTransformation();

  // calculate the distance and color error
  double distance_error = this->calculateRegistrationMetricDistanceError(candidate, aligned);
  double color_error = this->calculateRegistrationMetricOverlap(candidate, aligned);

  // calculate the final weighted result
  double result = ALPHA * (3.0 * distance_error) + (1.0 - ALPHA) * (color_error / 100.0);
  return result;
}

double PointCloudRecognizer::calculateRegistrationMetricDistanceError(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr base,
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target) const
{
  // search using a KD tree
  pcl::KdTreeFLANN<PointXYZRGB> search_tree;
  search_tree.setInputCloud(base);

  // search for the nearest point to each point
  double score = 0;
  for (size_t i = 0; i < target->size(); i++)
  {
    vector<int> indices;
    vector<float> distances;
    search_tree.nearestKSearch(target->at(i), 1, indices, distances);
    score += (double) distances[0];
  }

  return score;
}

double PointCloudRecognizer::calculateRegistrationMetricOverlap(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr base,
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target) const
{
  // search with a KD tree
  pcl::KdTreeFLANN<pcl::PointXYZRGB> search_tree;
  search_tree.setInputCloud(base);

  // search each point
  int score = 0;
  float error = 0;
  for (size_t i = 0; i < target->size(); i++)
  {
    // get the current point
    vector<int> indices;
    vector<float> distances;
    const pcl::PointXYZRGB &search_point = target->at(i);
    // perform a radius search to see how many neighbors are found
    int neighbors = search_tree.radiusSearch(search_point, OVERLAP_SEARCH_RADIUS, indices, distances);
    // check if there are enough neighbors
    if (neighbors > 0)
    {
      score++;
      // check the average RGB color distance
      double rgb_distance = 0;
      for (size_t j = 0; j < indices.size(); j++)
      {
        const pcl::PointXYZRGB &point = base->at(indices[j]);
        rgb_distance += sqrt(pow(search_point.r - point.r, 2) + pow(search_point.g - point.g, 2) +
            pow(search_point.b - point.b, 2));
      }
      // normalize the distance
      rgb_distance /= neighbors;
      error += rgb_distance;
    }
  }

  // normalize the error
  error /= ((double) score);
  return error;
}

void PointCloudRecognizer::transformGrasps(const Eigen::Matrix4f &icp_transform, const bool icp_swapped,
    const geometry_msgs::Point &centroid,
    const vector<rail_pick_and_place_msgs::GraspWithSuccessRate> &candidate_grasps,
    vector<rail_pick_and_place_msgs::GraspWithSuccessRate> &grasps) const
{
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
    const geometry_msgs::Pose &pose = candidate_grasps[i].grasp_pose.pose;
    tf2::Quaternion q_pose(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Vector3 v_pose(pose.position.x, pose.position.y, pose.position.z);
    tf2::Transform tf_pose(q_pose, v_pose);

    // push back the basic information
    grasps.push_back(candidate_grasps[i]);

    tf2::Transform result;
    if (icp_swapped)
    {
      tf2::Transform result;
      result.mult(tf_icp, tf_pose);
    } else
    {
      // use the invese for the result
      tf2::Transform inverse = tf_icp.inverse();
      result.mult(inverse, tf_pose);
    }

    // copy over the values
    grasps[i].grasp_pose.pose.position.x = result.getOrigin().x();
    grasps[i].grasp_pose.pose.position.y = result.getOrigin().y();
    grasps[i].grasp_pose.pose.position.z = result.getOrigin().z();
    grasps[i].grasp_pose.pose.orientation.x = result.getRotation().x();
    grasps[i].grasp_pose.pose.orientation.y = result.getRotation().y();
    grasps[i].grasp_pose.pose.orientation.z = result.getRotation().z();
    grasps[i].grasp_pose.pose.orientation.w = result.getRotation().w();

    // correct for the origin transform
    grasps[i].grasp_pose.pose.position.x += centroid.x;
    grasps[i].grasp_pose.pose.position.y += centroid.y;
    grasps[i].grasp_pose.pose.position.z += centroid.z;
  }
}
