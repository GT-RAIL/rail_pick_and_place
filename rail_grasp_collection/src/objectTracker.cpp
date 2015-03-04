#include <rail_grasp_collection/objectTracker.h>

using namespace std;

objectTracker::objectTracker()
{
  objectSubscriber = n.subscribe("rail_segmentation/segmented_objects", 1, &objectTracker::objectCallback, this);

  closestObjectServer = n.advertiseService("rail_grasp_collection/determine_closest_object", &objectTracker::determineClosestObject, this);
}

void objectTracker::objectCallback(const rail_manipulation_msgs::SegmentedObjectList &objects)
{
  ROS_INFO("Received new object data");
  objectList = objects;
  ROS_INFO("Updated object data");
}

bool objectTracker::determineClosestObject(rail_grasp_collection::ClosestObject::Request &req, rail_grasp_collection::ClosestObject::Response &res)
{
  unsigned int objectIndex;
  if (objectList.objects.size() == 0)
  {
    ROS_INFO("No objects detected...");
    return false;
  }
  else if (objectList.objects.size() == 1)
  {
    objectIndex = 0;
  }
  else
  {
    //find the graspable object with the lowest minimum distance
    //from the reqested point
    float minDst = 999;
    objectIndex = 0;
    for (unsigned int i = 0; i < objectList.objects.size(); i++)
    {
      float minPointDst = 999;
      //convert PointCloud2 to PointCloud to access the data
      sensor_msgs::PointCloud tempCloud;
      sensor_msgs::convertPointCloud2ToPointCloud(objectList.objects[i].cloud, tempCloud);
      for (unsigned int j = 0; j < tempCloud.points.size(); j++)
      {
        float dst = sqrt(
            pow(tempCloud.points[j].x - req.x, 2) +
                pow(tempCloud.points[j].y - req.y, 2) +
                pow(tempCloud.points[j].z - req.z, 2));
        if (dst < minPointDst)
          minPointDst = dst;
      }
      if (minPointDst < minDst)
      {
        minDst = minPointDst;
        objectIndex = i;
      }
    }
  }

  res.reference_frame_id = objectList.header.frame_id;
  sensor_msgs::convertPointCloud2ToPointCloud(objectList.objects[objectIndex].cloud, res.pointCloud);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracker");

  objectTracker ot;

  ros::spin();

  return 0;
}
