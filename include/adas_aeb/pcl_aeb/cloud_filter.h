#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3DArray.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>



class CloudFilter
{
    public:
        CloudFilter(const ros::NodeHandle&);
        ~CloudFilter(){}
        void callback(const sensor_msgs::PointCloud2ConstPtr& cloudIn);
        void paintClusters(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
        void downSample(pcl::PCLPointCloud2::Ptr PCLcloud);
        void stripStreetArea(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        void clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters);
        void publishFiltered(pcl::PCLPointCloud2::Ptr cloud);
        void publishClustered(pcl::PCLPointCloud2::Ptr cloud, const std::string &frame_id);
        void findBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Vector3f &bboxTransform, Eigen::Quaternionf &bboxQuaternion, pcl::PointXYZRGB minPoint, pcl::PointXYZRGB maxPoint);

    private:
        ros::NodeHandle handle;
        ros::Subscriber sub;
        ros::Publisher pub_stopFlag;
        ros::Publisher pub_filtered;
        ros::Publisher pub_clustered;
        ros::Publisher pub_bbox;

        float leafSize = 0.01;
};
