#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <wpi_jaco_msgs/ExecuteGraspAction.h>
#include <wpi_jaco_msgs/ExecutePickupAction.h>
#include <rail_grasp_collection/PickupAction.h>
#include <rail_grasp_collection/ClosestObject.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <time.h>

#define NUM_ARM_JOINTS 6
#define JOINT_START_INDEX 0
#define NUM_FINGERS 3
#define FINGER_START_INDEX 6

class graspCollection
{
public:	
	std::vector<double> rArmJoints;
	std::vector<double> lArmJoints;
	std::vector<double> armJoints;
	std::vector<double> fingerJoints;

	ros::NodeHandle n;
	
	ros::Publisher rArmPublisher;
	ros::Publisher lArmPublisher;
	//debug
	ros::Publisher debugPublisher;
	//end debug
	
	ros::Subscriber jointStateSubscriber;
	
	//Temporarily unused as the JACO arm does not have a good method of grasp verification
	//ros::ServiceClient verifyGraspClient;
	ros::ServiceClient closestObjectClient;
	
	//action clients for arm movement
	actionlib::SimpleActionClient<wpi_jaco_msgs::ExecuteGraspAction> acGrasp;
	actionlib::SimpleActionClient<wpi_jaco_msgs::ExecutePickupAction> acPickup;
	
	//action server
	actionlib::SimpleActionServer<rail_grasp_collection::PickupAction> as;
	std::string actionName;
	rail_grasp_collection::PickupFeedback asFeedback;
	rail_grasp_collection::PickupResult asResult;
	
	tf::TransformListener listener;
	
	/**
	* Callback for the actionlib server for picking up an object
	*/
	void executePickup(const rail_grasp_collection::PickupGoalConstPtr& goal);
	
	/**
	* Callback to get joint state information
	*/
	void jointStatesCallback(const sensor_msgs::JointState& states);
		
	/**
	* Constructor
	*/
	graspCollection(std::string name);

private:
	/**
	 * Helper function to convert tf::StampedTransform to geometry_msgs::PoseStamped
	 */
	void convertTFTransformToPoseMsg(const tf::StampedTransform &transform, geometry_msgs::PoseStamped &pose);
};
