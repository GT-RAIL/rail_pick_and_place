// RAIL Recognition
#include "rail_recognition/MetricTrainer.h"
#include "rail_recognition/PointCloudMetrics.h"

using namespace std;
using namespace rail::pick_and_place;

MetricTrainer::MetricTrainer()
    : private_node_("~")
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

  train_metric_server_ = private_node_.advertiseService("train_metrics", &MetricTrainer::trainMetrics, this);

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


bool MetricTrainer::trainMetrics(rail_pick_and_place_msgs::TrainMetrics::Request &req, rail_pick_and_place_msgs::TrainMetrics::Response &res)
{
  ROS_INFO("Gathering metrics for %s.  Check the rviz window to see the matches, and use this command line to give feedback.", req.object_name.c_str());

  // get all of the grasp demonstrations for the given object name
  vector<PCLGraspModel> grasp_models;
  vector<graspdb::GraspDemonstration> demonstrations;
  graspdb_->loadGraspDemonstrationsByObjectName(req.object_name, demonstrations);
  for (size_t i = 0; i < demonstrations.size(); i++)
  {
    // translate the demonstration into a grasp model
    graspdb::GraspModel model;
    model.setPointCloud(demonstrations[i].getPointCloud());
    model.setObjectName(demonstrations[i].getObjectName());
    graspdb::Grasp grasp;
    grasp.setGraspPose(demonstrations[i].getGraspPose());
    grasp.setEefFrameID(demonstrations[i].getEefFrameID());
    model.addGrasp(grasp);

    // convert to a PCL version of the grasp model
    PCLGraspModel pcl_grasp_model(model);
    grasp_models.push_back(pcl_grasp_model);
  }

  // try merging every combination of grasps and gather metrics for each
  if (grasp_models.size() > 1)
  {
    ofstream outputFile;
    outputFile.open("registration_metrics.txt", ios::out | ios::app);

    for (unsigned int i = 0; i < grasp_models.size() - 1; i ++)
    {
      for (unsigned int j = i + 1; j < grasp_models.size(); j ++)
      {
        ROS_INFO("Merging point clouds %d and %d...", i, j);

        PCLGraspModel base;
        PCLGraspModel target;
        // set the larger point cloud as the base point cloud
        if (grasp_models[i].getPointCloud().data.size() >= grasp_models[j].getPointCloud().data.size())
        {
          base = grasp_models[i];
          target = grasp_models[j];
        }
        else
        {
          base = grasp_models[j];
          target = grasp_models[i];
        }

        // convert to pcl point clouds
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &base_pc = base.getPCLPointCloud();
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_pc = target.getPCLPointCloud();

        // perform ICP on the point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
        point_cloud_metrics::performICP(base_pc, target_pc, aligned_pc);

        // publish result for human verification
        base_pc_pub_.publish(base_pc);
        aligned_pc_pub_.publish(aligned_pc);

        //calculate metrics
        double m_o = point_cloud_metrics::calculateRegistrationMetricOverlap(base_pc, aligned_pc, false);
        double m_d_err = point_cloud_metrics::calculateRegistrationMetricDistanceError(base_pc, aligned_pc);
        double m_c_err = point_cloud_metrics::calculateRegistrationMetricOverlap(base_pc, aligned_pc, true);
        double m_c_avg = point_cloud_metrics::calculateRegistrationMetricColorRange(base_pc, aligned_pc);
        double m_c_dev = point_cloud_metrics::calculateRegistrationMetricStdDevRange(base_pc, aligned_pc);
        double m_spread = point_cloud_metrics::calculateRegistrationMetricDistance(base_pc, aligned_pc);

        //wait for command line input denoting positive or negative registration
        char c;
        cout << "Is this a successful merge? (y/n) " << endl;
        do {
          c = getchar();
          putchar(c);
        } while (c != 'y' || c != 'n');

        ROS_INFO("Writing data...");

        //write
        outputFile << m_o << "," << m_d_err << "," << m_c_err << "," << m_c_avg << "," << m_c_dev << "," << m_spread << "," << c << "\n";
        if (c == 'y')
          ROS_INFO("Wrote positive merge data point.");
        else
          ROS_INFO("Wrote negative merge data point.");
      }
    }

    outputFile.close();

    ROS_INFO("All pairs attempted, metric training data gathering complete.");
  }

  return true;
}
