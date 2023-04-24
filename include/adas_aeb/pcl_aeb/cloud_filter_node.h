#include "adas_aeb/pcl_aeb/cloud_filter.h"
#include "adas_utils/ConfigParser.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>

class CloudFilterNode
{
    public:
        CloudFilterNode(adas::ConfigParser& parser, const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
        ~CloudFilterNode(){};
        void callback(const sensor_msgs::PointCloud2ConstPtr& cloudIn);
        void createMarker(visualization_msgs::Marker &marker, bool inRoi, unsigned long id, Bbox &box);
        vision_msgs::BoundingBox3D& toRosBBox(Bbox &input_box);
        void publishClustered(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string &frame_id);

    private:
        ros::NodeHandle handle;
        ros::NodeHandle privateHandle;

        ros::Subscriber sub;
        ros::Publisher pub_stopFlag;
        ros::Publisher pub_clustered;
        ros::Publisher pub_bbox;
        ros::Publisher pub_markers;

        CloudFilter cloudFilter;
};