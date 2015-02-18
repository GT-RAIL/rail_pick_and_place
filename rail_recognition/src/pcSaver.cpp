#include "rail_recognition/pcSaver.h"

using namespace std;
using namespace pcl;

pcSaver::pcSaver()
{
  count = 1;

  cloudPublisher = n.advertise<sensor_msgs::PointCloud2>("pc_saver/cloud", 1);

  segmentationClient = n.serviceClient<rail_segmentation::Segment>("/rail_segmentation/segment");

  segmentationServer = n.advertiseService("pc_saver/segment", &pcSaver::segmentService, this);
}

bool pcSaver::segmentService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  rail_segmentation::Segment srv;

  if (!segmentationClient.call(srv))
  {
    ROS_INFO("Couldn't call segmentation service.");
    return false;
  }
  if (srv.response.objects.empty())
  {
    ROS_INFO("No segmented objects found.");
    return true;
  }

  //save point clouds
  clouds = srv.response.objects;
  for (unsigned int i = 0; i < clouds.size(); i++)
  {
    ROS_INFO("Saving point cloud %d...", count);

    //convert to pcl point cloud
    PointCloud<PointXYZRGB>::Ptr pclCloudPtr(new PointCloud<PointXYZRGB>);
    PCLPointCloud2 tempCloud;
    pcl_conversions::toPCL(clouds[i], tempCloud);
    fromPCLPointCloud2(tempCloud, *pclCloudPtr);

    //save as pcd file
    stringstream ss;
    ss.str("");
    ss << "cloud_" << count << ".pcd";
    io::savePCDFileASCII(ss.str(), *pclCloudPtr);

    count++;
  }

  ROS_INFO("Finished.");
  return true;
}

void pcSaver::publishClouds()
{
  //publish the first point cloud
  if (!clouds.empty())
  {
    cloudPublisher.publish(clouds[0]);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_saver");

  pcSaver pcs;

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    pcs.publishClouds();
    loop_rate.sleep();
  }

  return 0;
}
