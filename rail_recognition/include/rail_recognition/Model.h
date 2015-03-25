#ifndef MODEL_H_
#define MODEL_H_

#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rail_manipulation_msgs/SegmentedObject.h>
#include <rail_pick_and_place_msgs/GraspDemonstration.h>
#include <rail_pick_and_place_msgs/GraspModel.h>
#include <rail_pick_and_place_msgs/GraspWithSuccessRate.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_conversions/pcl_conversions.h>

namespace rail
{
namespace pick_and_place
{

class Model
{
public:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
  std::vector<rail_pick_and_place_msgs::GraspWithSuccessRate> graspList;
  std::string objectName;
  uint32_t modelID;

  Model()
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    this->pointCloud = newCloud;
    objectName = "";
  }

  void copy(const Model &in)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopyPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> &cloudCopy = *cloudCopyPtr;
    cloudCopy = *(in.pointCloud);
    this->pointCloud = cloudCopyPtr;
    this->graspList = in.graspList;
    this->objectName = in.objectName;
    this->modelID = in.modelID;
  }

  void copyFromGraspDemonstrationMsg(const rail_pick_and_place_msgs::GraspDemonstration &demonstration)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopyPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> &cloudCopy = *cloudCopyPtr;
    pcl::PCLPointCloud2 tempConvCloud;
    pcl_conversions::toPCL(demonstration.point_cloud, tempConvCloud);
    pcl::fromPCLPointCloud2(tempConvCloud, cloudCopy);
    this->pointCloud = cloudCopyPtr;
    rail_pick_and_place_msgs::GraspWithSuccessRate grasp;
    grasp.grasp_pose = demonstration.grasp_pose;
    grasp.attempts = 0;
    grasp.successes = 0;
    grasp.eef_frame_id = demonstration.eef_frame_id;
    this->graspList.push_back(grasp);
    this->objectName = demonstration.object_name;
    this->modelID = demonstration.id;
  }

  void copyFromGraspModelMsg(const rail_pick_and_place_msgs::GraspModel &model)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopyPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> &cloudCopy = *cloudCopyPtr;
    pcl::PCLPointCloud2 tempConvCloud;
    pcl_conversions::toPCL(model.point_cloud, tempConvCloud);
    pcl::fromPCLPointCloud2(tempConvCloud, cloudCopy);
    this->pointCloud = cloudCopyPtr;
    this->graspList = model.grasps;
    this->objectName = model.object_name;
    this->modelID = model.id;
  }

  void copyFromSegmentedObjectMsg(const rail_manipulation_msgs::SegmentedObject &object)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopyPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB> &cloudCopy = *cloudCopyPtr;
    pcl::PCLPointCloud2 tempConvCloud;
    pcl_conversions::toPCL(object.point_cloud, tempConvCloud);
    pcl::fromPCLPointCloud2(tempConvCloud, cloudCopy);
    this->pointCloud = cloudCopyPtr;
    this->graspList.resize(object.grasps.size());
    for (unsigned int i = 0; i < this->graspList.size(); i ++)
    {
      graspList[i].grasp_pose = object.grasps[i];
    }
    this->objectName = object.name;
    this->modelID = object.model_id;
  }
};

} //end namespace pick_and_place
} //end namespace rail

#endif
