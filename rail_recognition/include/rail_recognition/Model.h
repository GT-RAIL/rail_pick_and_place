#include <geometry_msgs/Pose.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rail_recognition
{
  class Model
  {
  public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
    std::vector<geometry_msgs::Pose> graspList;
    std::vector<int> successesList;
    std::vector<int> totalAttemptsList;

    Model()
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      this->pointCloud = newCloud;
    }

    void copy(const rail_recognition::Model& in)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopyPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB> &cloudCopy = *cloudCopyPtr;
      cloudCopy = *(in.pointCloud);
      this->pointCloud = cloudCopyPtr;
      this->graspList = in.graspList;
      this->successesList = in.successesList;
      this->totalAttemptsList = in.totalAttemptsList;
    }
  };
}
