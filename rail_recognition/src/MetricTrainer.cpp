/*!
 * \file MetricTrainer.cpp
 * \brief The metric trainer node object.
 *
 * The metric trainer allows for generating data sets for training registration metric decision trees. An action server
 * is used to provide the object name and files are dumped to "registration_metrics.txt".
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \author David Kent, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

// RAIL Recognition
#include "rail_recognition/MetricTrainer.h"
#include "rail_recognition/PointCloudMetrics.h"

// ROS
#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace rail::pick_and_place;

MetricTrainer::MetricTrainer()
    : private_node_("~"), get_yes_and_no_feedback_ac_(private_node_, "get_yes_no_feedback", true),
      as_(private_node_, "train_metrics", boost::bind(&MetricTrainer::trainMetricsCallback,
                                                      this, _1), false)
{
  // set defaults
  int port = graspdb::Client::DEFAULT_PORT;
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");

  // grab any parameters we need
  node_.getParam("/graspdb/host", host);
  node_.getParam("/graspdb/port", port);
  node_.getParam("/graspdb/user", user);
  node_.getParam("/graspdb/password", password);
  node_.getParam("/graspdb/db", db);

  // connect to the grasp database
  graspdb_ = new graspdb::Client(host, port, user, password, db);
  okay_ = graspdb_->connect();

  // setup the point cloud publishers
  base_pc_pub_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("base_pc", 1, true);
  aligned_pc_pub_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("aligned_pc", 1, true);

  if (okay_)
  {
    ROS_INFO("Metric Trainer Successfully Initialized");
    as_.start();
  }
}

MetricTrainer::~MetricTrainer()
{
  // cleanup
  as_.shutdown();
  graspdb_->disconnect();
  delete graspdb_;
}

bool MetricTrainer::okay() const
{
  return okay_;
}

void MetricTrainer::trainMetricsCallback(const rail_pick_and_place_msgs::TrainMetricsGoalConstPtr &goal)
{
  ROS_INFO("Gathering metrics for %s. Check RViz to see the matches.", goal->object_name.c_str());

  // default to false
  rail_pick_and_place_msgs::TrainMetricsFeedback feedback;
  rail_pick_and_place_msgs::TrainMetricsResult result;
  result.success = false;

  // get all of the grasp demonstrations for the given object name
  feedback.message = "Loading grasp demonstrations...";
  as_.publishFeedback(feedback);
  vector<graspdb::GraspDemonstration> demonstrations;
  graspdb_->loadGraspDemonstrationsByObjectName(goal->object_name, demonstrations);

  // try merging every combination of grasps and gather metrics for each
  if (demonstrations.size() >= 2)
  {
    // convert to PCL point clouds and filter them
    feedback.message = "Converting to PCL point clouds...";
    as_.publishFeedback(feedback);
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> point_clouds;
    for (size_t i = 0; i < demonstrations.size(); i++)
    {
      // create the PCL point cloud
      point_clouds.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>));
      // convert from a ROS message
      point_cloud_metrics::rosPointCloud2ToPCLPointCloud(demonstrations[i].getPointCloud(), point_clouds[i]);
      // filter and move to the origin
      point_cloud_metrics::filterPointCloudOutliers(point_clouds[i]);
      point_cloud_metrics::transformToOrigin(point_clouds[i]);
    }

    // create the output file
    ofstream output_file;
    output_file.open("registration_metrics.txt", ios::out | ios::app);

    // check all pairs
    for (size_t i = 0; i < point_clouds.size() - 1; i++)
    {
      for (size_t j = i + 1; j < point_clouds.size(); j++)
      {
        stringstream ss;
        ss << i << " and " << j;
        string i_j_str = ss.str();
        feedback.message = "Merging point clouds " + i_j_str + "...";
        as_.publishFeedback(feedback);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_pc;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pc;

        // set the larger point cloud as the base point cloud
        if (point_clouds[i]->size() > point_clouds[j]->size())
        {
          base_pc = point_clouds[i];
          target_pc = point_clouds[j];
        }
        else
        {
          base_pc = point_clouds[j];
          target_pc = point_clouds[i];
        }

        // perform ICP on the point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
        point_cloud_metrics::performICP(base_pc, target_pc, aligned_pc);

        // publish result for human verification
        base_pc_pub_.publish(base_pc);
        aligned_pc_pub_.publish(aligned_pc);

        // wait for input denoting positive or negative registration
        feedback.message = "Getting metrics for point clouds " + i_j_str + "...";
        as_.publishFeedback(feedback);
        // send the request
        rail_pick_and_place_msgs::GetYesNoFeedbackGoal goal;
        get_yes_and_no_feedback_ac_.sendGoal(goal);

        // calculate all metrics
        double m_o, m_c_err;
        point_cloud_metrics::calculateRegistrationMetricOverlap(base_pc, aligned_pc, m_o, m_c_err);
        double m_d_err = point_cloud_metrics::calculateRegistrationMetricDistanceError(base_pc, aligned_pc);

        // wait for a response
        feedback.message = "Waiting for feedback on point clouds " + i_j_str + "...";
        as_.publishFeedback(feedback);
        get_yes_and_no_feedback_ac_.waitForResult();
        string input = (get_yes_and_no_feedback_ac_.getResult()->yes) ? "y" : "n";

        // write the data to the file
        output_file << m_o << "," << m_d_err << "," << m_c_err << "," << input << endl;
      }
    }

    // save the file and finish
    output_file.close();
    result.success = true;
    as_.setSucceeded(result);
  } else
  {
    // not enough models
    string message = "Less than 2 models found for " + goal->object_name + ", ignoring request.";
    ROS_WARN("%s", message.c_str());
    as_.setSucceeded(result, message);
  }
}
