#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3DArray.h>
#include <visualization_msgs/MarkerArray.h>
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
#include <opencv2/opencv.hpp>
#include "adas_utils/ConfigParser.h"

//to be moved to msgs types
struct Bbox
{
    cv::RotatedRect XYPlane;
    cv::Point2f zBound; // x - min, y - max

    Bbox(cv::RotatedRect &XY, cv::Point2f &z){
        XYPlane = XY;
        zBound = z;
    }
    Bbox(){
        XYPlane = cv::RotatedRect();
        zBound = cv::Point2f();
    }
};

class CloudFilter
{
    public:
        CloudFilter(adas::ConfigParser& parser, const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
        ~CloudFilter(){}
        void setROI();
        void callback(const sensor_msgs::PointCloud2ConstPtr& cloudIn);
        void getClusters(const sensor_msgs::PointCloud2 &cloud_msg, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr rgbCloud);
        void downSample(pcl::PCLPointCloud2::Ptr PCLcloud);
        void stripStreetArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters);
        void publishClustered(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string &frame_id);
        void findBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Bbox &box);
        bool isBoxInROI(Bbox &box);
        vision_msgs::BoundingBox3D& toRosBBox(Bbox &input_box);
        void createMarker(visualization_msgs::Marker &marker, bool inRoi, unsigned long id, Bbox &box);

    private:
        ros::NodeHandle handle;
        ros::NodeHandle privateHandle;

        ros::Subscriber sub;
        ros::Publisher pub_stopFlag;
        ros::Publisher pub_clustered;
        ros::Publisher pub_bbox;
        ros::Publisher pub_markers;

        float leafSize;
        float ROI_min_x;
        float ROI_max_x;
        float ROI_y;
        float street_level;
        float cluster_tolerance;
        int min_cluster_size;
        int max_cluster_size;

        std::vector<cv::Point2f> ROI;
};


