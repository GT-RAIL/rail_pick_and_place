#include "rail_recognition/GraspReader.h"

using namespace std;

GraspReader::GraspReader()
{
  // private node handle
  ros::NodeHandle private_nh("~");

  //set location of models
  stringstream ss;
  ss << ros::package::getPath("rail_pick_and_place_tools") << "/models";
  private_nh.param("model_dir", modelDirectory, ss.str());

  //Advertise and subscribe to everything required
  readGraspServer = n.advertiseService("grasp_reader/read_grasps", &GraspReader::readGraspService, this);

  cloudPublisher = n.advertise<sensor_msgs::PointCloud>("grasp_reader/point_cloud", 1);
  graspsPublisher = n.advertise<geometry_msgs::PoseArray>("grasp_reader/grasps", 1);
}

bool GraspReader::readGraspService(rail_recognition::ReadGrasp::Request &req, rail_recognition::ReadGrasp::Response &res)
{
  rail_grasping::GraspModel result;

  stringstream ss;
  ss << modelDirectory << "/" << req.grasp_entry;

  ifstream inFile(ss.str().c_str());

  if (inFile.is_open())
  {
    string line;
    while (inFile.good())
    {
      //read reference frame
      getline(inFile, line);
      string str("reference_frame_id: ");
      while (inFile.good() && line.find(str) == string::npos)
        getline(inFile, line);
      line.erase(0, line.find(str) + str.length());
      result.reference_frame_id = line;

      //read pointcloud
      sensor_msgs::PointCloud tempCloud;

      str = "pointCloud:";
      while (inFile.good() && line.find(str) == string::npos)
        getline(inFile, line);

      str = "header:";
      while (inFile.good() && line.find(str) == string::npos)
        getline(inFile, line);

      str = "frame_id: ";
      while (inFile.good() && line.find(str) == string::npos)
        getline(inFile, line);
      line.erase(0, line.find(str) + str.length());
      tempCloud.header.frame_id = line;

      str = "points: ";
      while (inFile.good() && line.find(str) == string::npos)
        getline(inFile, line);
      line.erase(0, line.find(str) + str.length());
      if (line[0] == '[')
      {
        line.erase(line.begin());
        while (line[0] != ']')
        {
          line.erase(line.begin());
          geometry_msgs::Point32 point;
          point.x = atof(line.substr(0, line.find(',')).c_str());
          line.erase(0, line.find(',') + 1);
          point.y = atof(line.substr(0, line.find(',')).c_str());
          line.erase(0, line.find(',') + 1);
          point.z = atof(line.substr(0, line.find(']')).c_str());
          line.erase(0, line.find(']') + 1);
          tempCloud.points.push_back(point);
          if (line[0] == ',')
            line.erase(line.begin());
        }
      }

      sensor_msgs::ChannelFloat32 channel;

      str = "channels:";
      while (inFile.good() && line.find(str) == string::npos)
        getline(inFile, line);

      str = "name: ";
      while (inFile.good() && line.find(str) == string::npos)
        getline(inFile, line);
      line.erase(0, line.find(str) + str.length());
      channel.name = line;

      str = "values: ";
      while (inFile.good() && line.find(str) == string::npos)
        getline(inFile, line);
      line.erase(0, line.find(str) + str.length());
      if (line[0] == '[')
      {
        line.erase(line.begin());
        while (line[0] != ']')
        {
          if (line.find(',') != string::npos)
          {
            long int temp = atol(line.substr(0, line.find(',')).c_str());
            float value = *(float *) (&temp);
            channel.values.push_back(value);
            line.erase(0, line.find(',') + 1);
          }
          else
          {
            long int temp = atol(line.substr(0, line.find(']')).c_str());
            float value = *(float *) (&temp);
            channel.values.push_back(value);
            line.erase(0, line.find(']'));
          }
        }
        tempCloud.channels.push_back(channel);
      }

      result.pointCloud = tempCloud;

      cloudPublisher.publish(tempCloud);

      //read gripper poses
      while (inFile.good())
      {
        geometry_msgs::Pose tempPose;
        str = "gripperPose:";
        while (inFile.good() && line.find(str) == string::npos)
          getline(inFile, line);

        str = "position: ";
        while (inFile.good() && line.find(str) == string::npos)
          getline(inFile, line);
        line.erase(0, line.find(str) + str.length());
        if (line[0] == '[')
        {
          line.erase(line.begin());
          tempPose.position.x = atof(line.substr(0, line.find(',')).c_str());
          line.erase(0, line.find(',') + 1);
          tempPose.position.y = atof(line.substr(0, line.find(',')).c_str());
          line.erase(0, line.find(',') + 1);
          tempPose.position.z = atof(line.substr(0, line.find(']')).c_str());
        }

        str = "orientation: ";
        while (inFile.good() && line.find(str) == string::npos)
          getline(inFile, line);
        line.erase(0, line.find(str) + str.length());
        if (line[0] == '[')
        {
          line.erase(line.begin());
          tempPose.orientation.x = atof(line.substr(0, line.find(',')).c_str());
          line.erase(0, line.find(',') + 1);
          tempPose.orientation.y = atof(line.substr(0, line.find(',')).c_str());
          line.erase(0, line.find(',') + 1);
          tempPose.orientation.z = atof(line.substr(0, line.find(',')).c_str());
          line.erase(0, line.find(',') + 1);
          tempPose.orientation.w = atof(line.substr(0, line.find(']')).c_str());

          result.gripperPoses.push_back(tempPose);
        }
      }

      break;
    }
    inFile.close();

    //publish grasp poses for debugging
    if (req.visualize_grasps)
    {
      geometry_msgs::PoseArray graspsToPublish;
      graspsToPublish.header.frame_id = "base_footprint";
      graspsToPublish.poses = result.gripperPoses;
      for (unsigned int i = 0; i < graspsToPublish.poses.size(); i++)
      {
        if (i >= individualGraspPublishers.size())
        {
          stringstream ssTopic;
          ssTopic << "grasp_reader/grasp" << (i + 1);
          ros::Publisher newGraspPublisher = n.advertise<geometry_msgs::PoseStamped>(ssTopic.str(), 1);
          individualGraspPublishers.push_back(newGraspPublisher);
        }
        //setup grasp for visualization
        tf::Transform graspTransform;
        graspTransform.setOrigin(tf::Vector3(graspsToPublish.poses[i].position.x, graspsToPublish.poses[i].position.y, graspsToPublish.poses[i].position.z));
        graspTransform.setRotation(tf::Quaternion(graspsToPublish.poses[i].orientation.x, graspsToPublish.poses[i].orientation.y, graspsToPublish.poses[i].orientation.z, graspsToPublish.poses[i].orientation.w));

        ros::Time now = ros::Time::now();
        stringstream ssGraspFrame;
        ssGraspFrame << "grasp_frame" << i;
        tfBroadcaster.sendTransform(tf::StampedTransform(graspTransform, now, "base_footprint", ssGraspFrame.str()));
        tfListener.waitForTransform("/base_footprint", ssGraspFrame.str(), now, ros::Duration(1.0));

        geometry_msgs::PoseStamped poseToPublish;
        //approachPose.header.stamp = ros::Time::now();
        poseToPublish.header.frame_id = ssGraspFrame.str();
        poseToPublish.pose.position.x = -0.05;

        individualGraspPublishers[i].publish(poseToPublish);
      }
      graspsPublisher.publish(graspsToPublish);
    }

    ifstream inFile2(ss.str().c_str());

    if (inFile2.is_open())
    {
      string line;
      if (inFile2.good())
      {
        //read grasp successes
        getline(inFile2, line);
        string str("successes: ");
        while (inFile2.good() && line.find(str) == string::npos)
          getline(inFile2, line);
        line.erase(0, line.find(str) + str.length());
        if (line[0] == '[')
        {
          line.erase(line.begin());
          while (line[0] != ']')
          {
            if (line.find(',') != string::npos)
            {
              result.successes.push_back(atoi(line.substr(0, line.find(',')).c_str()));
              line.erase(0, line.find(',') + 1);
            }
            else
            {
              result.successes.push_back(atoi(line.substr(0, line.find(']')).c_str()));
              line.erase(0, line.find(']'));
            }
          }
        }
        while (result.gripperPoses.size() > result.successes.size())
        {
          result.successes.push_back(0);
        }

        str = "totalAttempts: ";
        while (inFile2.good() && line.find(str) == string::npos)
          getline(inFile2, line);
        line.erase(0, line.find(str) + str.length());
        if (line[0] == '[')
        {
          line.erase(line.begin());
          while (line[0] != ']')
          {
            if (line.find(',') != string::npos)
            {
              result.totalAttempts.push_back(atoi(line.substr(0, line.find(',')).c_str()));
              line.erase(0, line.find(',') + 1);
            }
            else
            {
              result.totalAttempts.push_back(atoi(line.substr(0, line.find(']')).c_str()));
              line.erase(0, line.find(']'));
            }
          }
        }
        while (result.gripperPoses.size() > result.totalAttempts.size())
        {
          result.totalAttempts.push_back(0);
        }
      }

      inFile2.close();
    }

    res.grasp = result;
    res.success = true;
  }
  else
  {
    ROS_INFO_STREAM("File " << req.grasp_entry << " does not exist!");
    res.success = false;
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_reader");

  GraspReader gr;

  ros::spin();

  return 0;
}
