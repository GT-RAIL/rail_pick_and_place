// RAIL Recognition
#include "rail_recognition/ModelGenerator.h"
#include "rail_recognition/PointCloudMetrics.h"

// ROS
#include <geometry_msgs/PoseArray.h>
#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace rail::pick_and_place;

ModelGenerator::ModelGenerator()
    : private_node_("~"),
      as_(private_node_, "generate_models", boost::bind(&ModelGenerator::generateModelsCallback, this, _1), false)
{
  // set defaults
  debug_ = DEFAULT_DEBUG;
  int port = graspdb::Client::DEFAULT_PORT;
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");

  // grab any parameters we need
  private_node_.getParam("debug", debug_);
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
    debug_pc_pub_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("debug_pc", 1, true);
    debug_poses_pub_ = private_node_.advertise<geometry_msgs::PoseArray>("debug_poses", 1, true);
  }

  if (okay_)
  {
    ROS_INFO("Model Generator Successfully Initialized");
  }

  as_.start();
}

ModelGenerator::~ModelGenerator()
{
  // cleanup
  as_.shutdown();
  graspdb_->disconnect();
  delete graspdb_;
}

bool ModelGenerator::okay() const
{
  return okay_;
}


void ModelGenerator::generateModelsCallback(const rail_pick_and_place_msgs::GenerateModelsGoalConstPtr &goal)
{
  ROS_INFO("Model generation request received.");

  rail_pick_and_place_msgs::GenerateModelsResult result;
  rail_pick_and_place_msgs::GenerateModelsFeedback feedback;

  // load each grasp demonstration
  feedback.message = "Loading grasp demonstrations...";
  as_.publishFeedback(feedback);
  vector<PCLGraspModel> grasp_models;
  for (size_t i = 0; i < goal->grasp_demonstration_ids.size(); i++)
  {
    graspdb::GraspDemonstration demonstration;
    if (graspdb_->loadGraspDemonstration(goal->grasp_demonstration_ids[i], demonstration))
    {
      // translate the demonstration into a grasp model
      graspdb::GraspModel model;
      model.setPointCloud(demonstration.getPointCloud());
      model.setObjectName(demonstration.getObjectName());
      graspdb::Grasp grasp;
      grasp.setGraspPose(demonstration.getGraspPose());
      grasp.setEefFrameID(demonstration.getEefFrameID());
      model.addGrasp(grasp);

      // convert to a PCL version of the grasp model
      PCLGraspModel pcl_grasp_model(model);
      grasp_models.push_back(pcl_grasp_model);
    } else
    {
      ROS_WARN("Could not load grasp demonstration with ID %d.", goal->grasp_demonstration_ids[i]);
    }
  }

  // load each existing model
  feedback.message = "Loading grasp models...";
  as_.publishFeedback(feedback);
  for (size_t i = 0; i < goal->grasp_model_ids.size(); i++)
  {
    graspdb::GraspModel model;
    if (graspdb_->loadGraspModel(goal->grasp_model_ids[i], model))
    {
      // convert to a PCL version of the grasp model
      PCLGraspModel pcl_grasp_model(model);
      grasp_models.push_back(pcl_grasp_model);
    } else
    {
      ROS_WARN("Could not load grasp model with ID %d.", goal->grasp_model_ids[i]);
    }
  }

  // generate and store the models
  feedback.message = "Registering models, please wait...";
  as_.publishFeedback(feedback);
  this->generateAndStoreModels(grasp_models, goal->max_model_size, result.new_model_ids);

  // finished
  as_.setSucceeded(result, "Success!");
}

void ModelGenerator::generateAndStoreModels(vector<PCLGraspModel> &grasp_models, const int max_model_size,
    vector<uint32_t> &new_model_ids) const
{
  // filter each point cloud and move to the origin
  for (size_t i = 0; i < grasp_models.size(); i++)
  {
    // filter the resulting PC
    point_cloud_metrics::filterPointCloudOutliers(grasp_models[i].getPCLPointCloud());
    point_cloud_metrics::transformToOrigin(grasp_models[i].getPCLPointCloud(), grasp_models[i].getGrasps());
  }

  // attempt to pair models
  ROS_INFO("Searching graph for model matches...");
  bool finished = false;
  while (!finished)
  {
    // default to finished and update as needed
    finished = true;
    // only continue if we have two models to compare
    if (grasp_models.size() > 1)
    {
      // TODO randomly order the elements to increase variability
      //random_shuffle(grasp_models.begin(), grasp_models.end());

      // use int to prevent rollover and start the matching process by starting at the end
      for (int i = (int) grasp_models.size() - 1; i >= 1; i--)
      {
        // the size of the model list can change inside the inner for loop
        if (i < grasp_models.size())
        {
          // use int to prevent rollover and start from the second to last
          for (int j = i - 1; j >= 0; j--)
          {
            // keep track of the base and the target point clouds for use in ICP
            const PCLGraspModel &base = grasp_models[i];
            const PCLGraspModel &target = grasp_models[j];

            // check if the maximum size would be allowed
            if (base.getNumGrasps() + target.getNumGrasps() > max_model_size)
            {
              ROS_INFO("Skipping pair %d-%d as a merge would exceed the maximum model size.", i, j);
            }
            else
            {
              // check if the registration passes
              PCLGraspModel result;
              if (this->registrationCheck(base, target, result))
              {
                ROS_INFO("Registration match found for pair %d-%d.", i, j);
                // remove both models from the global list
                grasp_models.erase(grasp_models.begin() + i);
                grasp_models.erase(grasp_models.begin() + j);
                // add the new model
                grasp_models.push_back(result);

                // check if we are running debug
                if (debug_)
                {
                  // generate the pose array
                  geometry_msgs::PoseArray poses;
                  for (size_t i = 0; i < result.getNumGrasps(); i++)
                  {
                    const graspdb::Pose &pose = result.getGrasp(i).getGraspPose();
                    poses.header.frame_id = pose.getRobotFixedFrameID();
                    poses.poses.push_back(pose.toROSPoseMessage());
                  }
                  // publish the poses and the resulting merged point cloud
                  debug_poses_pub_.publish(poses);
                  debug_pc_pub_.publish(*result.getPCLPointCloud());
                }

                // move down the list since we now have a match
                i--;
                finished = false;
              }
            }
          }
        }
      }
    }
  }

  // remove any single-grasp (unmerged) models and existing models and save the rest
  for (int i = ((int) grasp_models.size()) - 1; i >= 0; i--)
  {
    if (grasp_models[i].getNumGrasps() <= 1 || grasp_models[i].getID() != graspdb::GraspModel::UNSET_ID)
    {
      grasp_models.erase(grasp_models.begin() + i);
    } else
    {
      // attempt to store it
      graspdb::GraspModel new_model = grasp_models[i].toGraspModel();
      if (graspdb_->addGraspModel(new_model))
      {
        ROS_INFO("Added new model to the database with ID %d.", new_model.getID());
        new_model_ids.push_back(new_model.getID());
      }
      else
      {
        ROS_WARN("Error inserting model into database.");
      }
    }
  }
  ROS_INFO("Point cloud merging resulted in %lu models.", grasp_models.size());
}

bool ModelGenerator::registrationCheck(const PCLGraspModel &base, const PCLGraspModel &target,
    PCLGraspModel &result) const
{
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &base_pc = base.getPCLPointCloud();
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_pc = target.getPCLPointCloud();
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &result_pc = result.getPCLPointCloud();

  // perform ICP on the point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
  tf2::Transform tf_icp = point_cloud_metrics::performICP(base_pc, target_pc, aligned_pc);

  // check if the match is valid
  if (point_cloud_metrics::classifyMerge(base_pc, aligned_pc))
  {
    // transform each target grasp
    for (size_t i = 0; i < target.getNumGrasps(); i++)
    {
      // convert to tf2 matrix
      const graspdb::Grasp &old_grasp = target.getGrasp(i);
      const graspdb::Pose &old_grasp_pose = old_grasp.getGraspPose();
      tf2::Transform tf_pose = old_grasp_pose.toTF2Transform();

      // perform the transform
      tf2::Transform tf_result = tf_icp * tf_pose;

      // add the new grasp
      graspdb::Pose new_pose(old_grasp_pose.getRobotFixedFrameID(), tf_result);
      graspdb::Grasp new_grasp(new_pose, graspdb::GraspModel::UNSET_ID, old_grasp.getEefFrameID(),
          old_grasp.getSuccesses(), old_grasp.getAttempts());
      result.addGrasp(new_grasp);
    }

    // add the base grasps
    for (size_t i = 0; i < base.getNumGrasps(); i++)
    {
      graspdb::Grasp grasp(base.getGrasp(i));
      // reset the ID
      grasp.setGraspModelID(graspdb::GraspModel::UNSET_ID);
      result.addGrasp(grasp);
    }

    // merge the two point clouds
    *result_pc = *base_pc + *aligned_pc;

    point_cloud_metrics::filterRedundantPoints(result_pc);
    // move to the origin
    point_cloud_metrics::transformToOrigin(result_pc, result.getGrasps());

    // set the final model parameters
    result.setObjectName(base.getObjectName());
    return true;
  } else
  {
    return false;
  }
}
