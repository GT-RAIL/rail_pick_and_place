// RAIL Recognition
#include "rail_recognition/MetricTrainer.h"
#include "rail_recognition/PointCloudMetrics.h"

// ROS
#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace rail::pick_and_place;

MetricTrainer::MetricTrainer() : private_node_("~")
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

  base_pc_pub_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("base_pc", 1, true);
  aligned_pc_pub_ = private_node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("aligned_pc", 1, true);

  train_metrics_srv_ = private_node_.advertiseService("train_metrics", &MetricTrainer::trainMetrics, this);

  if (okay_)
  {
    ROS_INFO("Metric Trainer Successfully Initialized");
  }
}

MetricTrainer::~MetricTrainer()
{
  // cleanup
  graspdb_->disconnect();
  delete graspdb_;
}

bool MetricTrainer::okay() const
{
  return okay_;
}

bool MetricTrainer::trainMetrics(rail_pick_and_place_msgs::TrainMetrics::Request &req,
    rail_pick_and_place_msgs::TrainMetrics::Response &res)
{
  ROS_INFO("Gathering metrics for %s. Check RViz to see the matches, and use this command line to give feedback.",
      req.object_name.c_str());

  // get all of the grasp demonstrations for the given object name
  vector<graspdb::GraspDemonstration> demonstrations;
  graspdb_->loadGraspDemonstrationsByObjectName(req.object_name, demonstrations);

  // try merging every combination of grasps and gather metrics for each
  if (demonstrations.size() >= 2)
  {
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > point_clouds;
    point_clouds.resize(demonstrations.size());
    for (unsigned int i = 0; i < point_clouds.size(); i ++)
    {
      point_cloud_metrics::rosPointCloud2ToPCLPointCloud(demonstrations[i].getPointCloud(), point_clouds[i]);
      point_cloud_metrics::filterPointCloudOutliers(point_clouds[i]);
      point_cloud_metrics::transformToOrigin(point_clouds[i]);
    }

    ofstream output_file;
    output_file.open("registration_metrics.txt", ios::out | ios::app);

    for (size_t i = 0; i < point_clouds.size() - 1; i++)
    {
      for (size_t j = i + 1; j < point_clouds.size(); j++)
      {
        ROS_INFO("Merging point clouds %ld and %ld...", i, j);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

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

        // calculate all metrics
        double m_o = point_cloud_metrics::calculateRegistrationMetricOverlap(base_pc, aligned_pc, false);
        double m_d_err = point_cloud_metrics::calculateRegistrationMetricDistanceError(base_pc, aligned_pc);
        double m_c_err = point_cloud_metrics::calculateRegistrationMetricOverlap(base_pc, aligned_pc, true);
        double m_c_avg = point_cloud_metrics::calculateRegistrationMetricColorRange(base_pc, aligned_pc);
        double m_c_dev = point_cloud_metrics::calculateRegistrationMetricStdDevColorRange(base_pc, aligned_pc);
        double m_spread = point_cloud_metrics::calculateRegistrationMetricDistance(base_pc, aligned_pc);

        // wait for command line input denoting positive or negative registration
        string input;
        do
        {
          cout << "Is this a successful merge? (y/n) " << endl;
          cin >> input;
        } while (input != "y" && input != "n");

        // write the data to the file
        output_file << m_o << "," << m_d_err << "," << m_c_err << "," << m_c_avg << "," << m_c_dev << "," << m_spread
            << "," << input << endl;
      }
    }

    output_file.close();

    ROS_INFO("All pairs attempted, metric training data gathering complete.");
  } else
  {
    ROS_WARN("Less than 2 models found for '%s', ignoring request.", req.object_name.c_str());
  }

  return true;
}
