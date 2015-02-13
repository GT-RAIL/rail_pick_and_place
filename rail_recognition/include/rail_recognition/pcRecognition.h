//ROS
#include <ros/ros.h>
#include <rail_recognition/ReadGrasp.h>
#include <rail_pick_and_place_msgs/RecognizeAndGrasp.h>
#include <rail_pick_and_place_msgs/GraspRecognized.h>
#include <rail_recognition/Release.h>
#include <rail_segmentation/Recognize.h>
#include <rail_segmentation/SegmentedObjectList.h>
#include <rail_grasping/RequestGrasp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//C++
#include <boost/thread/thread.hpp>
#include <stdlib.h>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

//Point Cloud Filtering Constants
#define RADIUS .01
#define NUM_NEIGHBORS 23
#define DST_THRESHOLD .00075

//Recognition Constants
#define ALPHA .5

class pcRecognition
{
public:
	//ROS publishers, subscribers, and action servers
	ros::NodeHandle n;
	
	ros::Publisher releasePosePublisher;
	
	ros::Subscriber objectSubscriber;
	
  ros::ServiceServer recognizeServer;
	ros::ServiceServer recognizeAndGraspServer;
	ros::ServiceServer graspRecognizedServer;
	ros::ServiceServer setTrainingServer;
	ros::ServiceServer releaseServer;
	
	ros::ServiceClient readGraspClient;
	ros::ServiceClient requestGraspClient;
	ros::ServiceClient requestReleaseClient;
	
	//Point clouds
	sensor_msgs::PointCloud2 baseCloud;
	sensor_msgs::PointCloud2 targetCloud;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> models;
	
	//Point clouds in
	rail_segmentation::SegmentedObjectList objectList;
	
	//Grasp info
	std::vector< std::vector<geometry_msgs::Pose> > graspLists;
	float xTrans;
	float yTrans;
	float zTrans;
	std::vector< std::vector<int> > successesList;
	std::vector< std::vector<int> > totalAttemptsList;
	
	//grasp training info
	float epsilon;
	bool training;
	
	//tf
	tf::TransformListener tfListener;
	tf::TransformBroadcaster tfBroadcaster;
	
	//current state
	geometry_msgs::Pose currentGrasp;
	
	/**
	 * Constructor
	 */
	pcRecognition();
	
	/**
	 * Service callback to switch between training and testing modes, will also print out current training information
	 * to the terminal when a switch is requested
	 * @param req empty service request
	 * @param res empty service response
	 * @return true on successful service callback
	 */
	bool toggleTrainingMode(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	
	/**
	 * Place an object given a placement coordinate frame
	 * @param req service request including a coordinate frame to place an object
	 * @param res service response
	 * @return true on success
   */
	bool releaseObject(rail_recognition::Release::Request &req, rail_recognition::Release::Response &res);
	
	/**
	 * Attempt to recognize a given point cloud
	 * @param req service request including a segmented point cloud
	 * @param res service response denoting recognition success, object name, and possible grasp poses
	 * @return true on successful service callback
	 */
	bool recognize(rail_segmentation::Recognize::Request &req,
		rail_segmentation::Recognize::Response &res);
	
	/**
	 * Attempt to recognize and pickup an object for training or testing purposes
	 * @param req service request including a point cloud and a maximum number of grasp attempts
	 * @param res service response denoting recognition and pickup success as a boolean value
	 * @return true on successful service callback
	 */
	bool recognizeAndPickup(rail_pick_and_place_msgs::RecognizeAndGrasp::Request &req,
		rail_pick_and_place_msgs::RecognizeAndGrasp::Response &res);
  
  /**
	 * Pickup a previously recognized object
	 * @param req service request including previously calculated grasp list
	 * @param res service response
	 * @return true on successful service callback
	 */
  bool graspRecognized(rail_pick_and_place_msgs::GraspRecognized::Request &req, rail_pick_and_place_msgs::GraspRecognized::Response &res);
  
  /**
   * Order grasps based on either training or testing and execute grasps until one 
   * is successful or no grasps remain
   * @param index object model index
   * @param numAttempts number of grasps to attempt before terminating
   * @param grasps list of grasps to use
   * @return true on success, false if no grasps remain to attempt
   */
  bool chooseGrasp(int index, int numAttempts, std::vector<geometry_msgs::Pose> grasps);
	
	/**
	 * Read in point cloud models
	 */
	void readPointClouds();
	
	/**
	 * Determine if a grasp has already been attempted when using multiple attempts to grasp an object
	 * @param index index of the grasp in question
	 * @param list list of indices of previously attempted grasps
	 * @return true if the grasp has already been attempted
	 */
	bool graspAlreadyAttempted(int index, std::vector<int> list);
	
	/**
	 * Reads a point cloud from the given filename and converts it into pcl::PointXYZRGB format
	 * @param filename the name of the file to read the pointcloud from
	 * @param pointcloudOut pointer to the pcl point cloud where file will be read
	 * @param graspListOut pointer to list that will store grasps associated with the read point cloud
	 * @param successesListOut pointer to a list that will store the number of successful pickups for each grasp
	 * @param totalAttemptsOut pointer to a list that will store the number of attempted pickups for each grasp
	 * @return true on success, false if the file doesn't exist or is formatted incorrectly
	 */
	bool getCloud(std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudOut,
		std::vector<geometry_msgs::Pose> *graspListOut, std::vector<int> *successesListOut,
		std::vector<int> *totalAttemptsOut);
	
	/**
	 * Testing function to recognize a set of point clouds read in from text files
	 */
	void recognizePointClouds();
	
	/**
	 * Determine a score for the registration of two point clouds
	 * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
	 * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
	 * @return score representing the success of the registration, calculated as a weighted combination of color and shape metrics
	 */
	float scoreRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);
	
	/**
	 * Register two point clouds together using an iterative closest point algorithm
	 * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
	 * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
	 * @param baseGrasps list of grasps associated with the base point cloud
	 * @param targetGrasps list of grasps associated with the target point cloud
	 * @param resultGrasps pointer to a list where the combined grasps will be stored
	 * @param scoreFiltered option to only register point clouds if their score meets a given threshold, false by default
	 * @return resulting point cloud from the registration process
	 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr, std::vector<geometry_msgs::Pose> baseGrasps,
		std::vector<geometry_msgs::Pose> targetGrasps, std::vector<geometry_msgs::Pose> *resultGrasps,
		bool scoreFiltered = false);
	
	/**
	 * Calculate a metric for how successful the registration was based on distance error
	 * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
	 * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
	 * @return a score representing the success of the registration
	 */
	float calculateRegistrationMetricDstError(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);
	
	/**
	 * Calculate a metric for how successful the registration was based on overlap
	 * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
	 * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
	 * @return a score representing the success of the registration
	 */
	float calculateRegistrationMetricOverlap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr, float dstThreshold);
	
	/**
	 * Calculate a metric for how successful the registration was based on range of colors in each point cloud
	 * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
	 * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
	 * @return a score representing the success of the registration
	 */
	float calculateRegistrationMetricColorRange(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);
	
	/**
	 * Calculate a metric for how successful the registration was based on the respective spatial lengths of each point cloud
	 * @param baseCloudPtr pointer to the point cloud to which the target will be transformed
	 * @param targetCloudPtr pointer to the point cloud that will be transformed to the base cloud
	 * @return a score representing the success of the registration
	 */
	float calculateRegistrationMetricDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr baseCloudPtr,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCloudPtr);
	
	/**
	 * Filters point cloud outliers if they have less neighbors than the neighbor threshold within a given radius
	 * @param cloudPtr pointer to the point cloud to be filtered
	 * @param radius the radius to search within for neighbors
	 * @param numNeighborThreshold minimum number of neighbors required to keep a point
	 */
	void filterCloudOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, double radius, int numNeighborThreshold);
	
	/**
	 * Removes extra points that are within a given threshold of other points to keep the point cloud size manageable
	 * @param cloudPtr pointer to the point cloud to be filtered
	 * @param dstThreshold the minimum distance between neighboring points at which to keep a point
	 */
	void filterRedundentPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, double dstThreshold);
	
	/**
	 * Translates a point cloud so that its center point lies on the origin
	 * @param cloudPtr pointer to the point cloud to be translated
	 */
	void translateToOrigin(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::vector<geometry_msgs::Pose> *grasps);
	
};
