//ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <rail_pick_and_place_msgs/GraspRecognized.h>
#include <rail_pick_and_place_msgs/PickupObject.h>
#include <rail_pick_and_place_msgs/PickupSegmentedObject.h>
#include <rail_pick_and_place_msgs/RecognizeAndGrasp.h>
#include <rail_segmentation/Segment.h>
#include <rail_segmentation/SegmentedObjectList.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <wpi_jaco_msgs/HomeArmAction.h>

class pickAndPlace
{
public:
	//ROS publishers, subscribers, and action servers
	ros::NodeHandle n;
	
	ros::Subscriber objectSubscriber;
	
	ros::ServiceServer pickupServer;
	ros::ServiceServer pickupSegmentedServer;
	
	ros::ServiceClient recognizeAndGraspClient;
	ros::ServiceClient segmentClient;
	ros::ServiceClient graspRecognizedClient;
	
	actionlib::SimpleActionClient<wpi_jaco_msgs::HomeArmAction> acHome;
	
	rail_segmentation::SegmentedObjectList objectList;
	
	/**
	 * Constructor
	 */
	pickAndPlace();
	
	/**
	 * High level call to recognize and pickup a specified object
	 * @param req service request including the name of the object to be picked up
	 * @param res service response denoting pickup success as a boolean value
	 * @return true on successful service callback
	 */
	bool pickup(rail_pick_and_place_msgs::PickupObject::Request &req,
		rail_pick_and_place_msgs::PickupObject::Response &res);

  /**
	 * High level call to pickup an already segmented but unrecognized object
	 * @param req service request
	 * @param res service response denoting pickup success as a boolean value
	 * @return true on successful service callback
	 */
	bool pickupSegmented(rail_pick_and_place_msgs::PickupSegmentedObject::Request &req,
		rail_pick_and_place_msgs::PickupSegmentedObject::Response &res);

  /**
   * Callback for segmented object list
   * @param objectList the list of segmented objects
   */  
  void objectCallback(const rail_segmentation::SegmentedObjectList& list);
};
