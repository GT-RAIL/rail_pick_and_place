#include <rail_grasp_collection/graspLogger.h>

using namespace std;

graspLogger::graspLogger()
{
  graspNum = 1;
  
  graspSubscriber = n.subscribe("/rail_grasp_collection/result", 1, &graspLogger::graspCallback, this);
}

void graspLogger::graspCallback(const rail_grasp_collection::PickupActionResult pickupResult)
{
  if (pickupResult.result.success == false)
    return;
  stringstream ss;
  ss << "grasp_" << graspNum << ".txt";

  ofstream myfile;
  myfile.open (ss.str().c_str());
  
  myfile << "reference_frame_id: " << pickupResult.result.reference_frame_id << "\npointCloud:\n\theader:\n\t\tframe_id: " << pickupResult.result.pointCloud.header.frame_id << "\n\tpoints: [";
  for (unsigned int i = 0; i < pickupResult.result.pointCloud.points.size(); i ++)
  {
    myfile << "[" << pickupResult.result.pointCloud.points[i].x << "," << pickupResult.result.pointCloud.points[i].y << "," << pickupResult.result.pointCloud.points[i].z << "]";
    if (i < pickupResult.result.pointCloud.points.size() - 1)
      myfile << ",";
  }
  
  /*
  myfile << "]\n\trgb:\n\t\tvalues: [";
  for (unsigned int i = 0; i < pickupResult.result.pointCloud.points.size(); i ++)
  {
    int r;
    int g;
    int b;
    myfile << "[" << r << "," << g << "," << b << "]";
    if (i < pickupResult.result.pointCloud.points.size() - 1)
      myfile << ",";
  }
  myfile << "]";
  */
  
  myfile << "]\n\tchannels:";
  for (unsigned int i = 0; i < pickupResult.result.pointCloud.channels.size(); i ++)
  {
    myfile << "\n\t\tname: " << pickupResult.result.pointCloud.channels[i].name << "\n\t\tvalues: [";
    for (unsigned int j = 0; j < pickupResult.result.pointCloud.channels[i].values.size(); j ++)
    {
      //test code
      uint32_t value = *(uint32_t *)(&pickupResult.result.pointCloud.channels[i].values[j]);
      myfile << value;
      //end test code
      //myfile << pickupResult.result.pointCloud.channels[i].values[j];
      if (j < pickupResult.result.pointCloud.channels[i].values.size() - 1)
        myfile << ",";
    }
    myfile << "]";
  }
  
  myfile << "\ngripperPose: " << "\n\tposition: [" << pickupResult.result.gripperPose.pose.position.x << "," << pickupResult.result.gripperPose.pose.position.y << "," << pickupResult.result.gripperPose.pose.position.z << "]\n\torientation: [" << pickupResult.result.gripperPose.pose.orientation.x << "," << pickupResult.result.gripperPose.pose.orientation.y << "," << pickupResult.result.gripperPose.pose.orientation.z << "," << pickupResult.result.gripperPose.pose.orientation.w << "]";
  
  myfile.close();
  
  graspNum ++;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_logger");
  
  graspLogger gl;
  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
