#include <ros/ros.h>
#include <rail_segmentation/SegmentedObjectList.h>
#include <rail_grasp_collection/ClosestObject.h>
#include <sensor_msgs/point_cloud_conversion.h>

class objectTracker
{
public:
	ros::NodeHandle n;
	
	rail_segmentation::SegmentedObjectList objectList;
	
	//Services
	ros::ServiceServer closestObjectServer;
	
	//Subscribers
	ros::Subscriber objectSubscriber;
	
	/**
	* Constructor
	*/
	objectTracker();
	
	void objectCallback(const rail_segmentation::SegmentedObjectList& objects);
	
	bool determineClosestObject(rail_grasp_collection::ClosestObject::Request &req, rail_grasp_collection::ClosestObject::Response &res);
};
