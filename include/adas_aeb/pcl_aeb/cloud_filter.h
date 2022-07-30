#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <std_msgs/Bool.h>

class CloudFilter
{
public:
    CloudFilter(const ros::NodeHandle&, const ros::NodeHandle&);
    ~CloudFilter(){}
    void callback(const sensor_msgs::PointCloud2ConstPtr& cloudIn);
    void setOrigin();
    void setEdge();
    visualization_msgs::Marker createPointList();
    bool filteringPoints(const sensor_msgs::PointCloud2ConstPtr& cloudIn);
    bool isInBox();
private:
    ros::NodeHandle handle, privHandle;
    ros::Subscriber sub;
    ros::Publisher pub_marker;
    ros::Publisher pub_stopFlag;


    visualization_msgs::Marker marker;

    geometry_msgs::Point origin;
    geometry_msgs::Vector3 edge;
};