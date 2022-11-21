#include "adas_aeb/pcl_aeb/cloud_filter.h"

CloudFilter::CloudFilter(const ros::NodeHandle& nh)
    : handle(nh)
{
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    std::string paramTmp;

    this->handle.getParam("merged_pcl_topic", paramTmp);
    this->sub = handle.subscribe("/cloud_merged", 10, &CloudFilter::callback, this);
    
    this->handle.getParam("stop_topic", paramTmp);
    this->pub_stopFlag = this->handle.advertise<std_msgs::Bool>(paramTmp, 1, false);

    this->pub_cluster = this->handle.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1, false);

}

// big reafactoring needed
void CloudFilter::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

    //downsampling
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.01, 0.01, 0.01); //to be moved to cfg file
    sor.filter(*cloudFilteredPtr);

    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr(xyz_cloud);

    pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

    //removing street
    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(xyzCloudPtr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.02, 0.35); //to be moved to cfg
    pass.filter(*xyzCloudPtrFiltered);

    pcl::PCLPointCloud2 outputPCL;
    pcl::toPCLPointCloud2(*xyzCloudPtrFiltered, outputPCL);
    //
    // tbc
    //

    sensor_msgs::PointCloud2* output = new sensor_msgs::PointCloud2;
    sensor_msgs::PointCloud2ConstPtr  outputPtr(output);

    // Convert back to ros msg
    pcl_conversions::fromPCL(outputPCL, *output);
    
    output->header = cloud_msg->header;

    ROS_DEBUG("it works");
    pub_cluster.publish(*output);
}
