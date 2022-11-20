#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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




class CloudFilter
{
    public:
        CloudFilter(const ros::NodeHandle&);
        ~CloudFilter(){}
        void callback(const sensor_msgs::PointCloud2ConstPtr& cloudIn);

    private:
        ros::NodeHandle handle;
        ros::Subscriber sub;
        ros::Publisher pub_stopFlag;
        ros::Publisher pub_cluster;
};