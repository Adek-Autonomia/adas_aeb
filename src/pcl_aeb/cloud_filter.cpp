#include "adas_aeb/pcl_aeb/cloud_filter.h"

//FIXME:: correct random color assignment

CloudFilter::CloudFilter(const ros::NodeHandle& nh)
    : handle(nh)
{
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    std::string paramTmp;
    float fParamTmp;

    this->handle.getParam("leaf_size", fParamTmp);
     // (x,y,z)

    this->handle.getParam("merged_pcl_topic", paramTmp);
    this->sub = handle.subscribe("/cloud_merged", 10, &CloudFilter::callback, this);
    
    this->handle.getParam("stop_topic", paramTmp);
    this->pub_stopFlag = this->handle.advertise<std_msgs::Bool>(paramTmp, 1, false);

    // this->pub_filtered = this->handle.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1, false);
    this->pub_clustered = this->handle.advertise<sensor_msgs::PointCloud2>("/cloud_clustered", 1, false);
    // this->pub_segmented = this->handle.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1, false);

}

void CloudFilter::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    this->downSample(cloud);

    pcl::fromPCLPointCloud2(*cloud, *rgbCloud);

    this->paintWholeCloudWhite(rgbCloud);
    this->passthrough(rgbCloud);

    // pcl::toPCLPointCloud2(*rgbCloud, *cloud);
    // this->publishFiltered(cloud);

    this->clustering(rgbCloud);
    this->paintClusters();

    for(size_t i = 0; i<clusters.size(); i++)
    {
        for(auto& point: *clusters[i] )
        {
            clusteredCloud->points.push_back(point);
        }
    }

    pcl::toPCLPointCloud2(*clusteredCloud, *cloud);

    this->publishClustered(cloud, cloud_msg->header.frame_id);

    ROS_DEBUG("it works");
}

void CloudFilter::paintWholeCloudWhite(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    for(auto& point: *cloud)
    {
        point.rgb = 0xFFFFFF;
    }
}

void CloudFilter::paintClusters()
{
    for (size_t i = 0; i < clusters.size(); i++)
    {
        int32_t rgb = (static_cast<uint32_t>(rand()%255)<<16|static_cast<uint32_t>(rand()%255)<<8|static_cast<uint32_t>(rand()%255));
        for( auto& point : *clusters[i])
        {
            point.rgb = rgb;
        }
    }
}

void CloudFilter::downSample(pcl::PCLPointCloud2::Ptr PCLcloud)
{
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setLeafSize(this->leafSize, this->leafSize, this->leafSize);
    vg.setInputCloud(PCLcloud);
    vg.filter(*PCLcloud);
}

void CloudFilter::passthrough(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.02,FLT_MAX);
    pass.filter(*cloud);
}

void CloudFilter::clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    for(auto it=cluster_indices.begin(); it!= cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        for(auto clust_it = it->indices.begin(); clust_it!=it->indices.end(); ++clust_it)
        {
            temp->points.push_back(cloud->points[*clust_it]);
        }
        this->clusters.push_back(temp); 
    }
}

void CloudFilter::publishFiltered(pcl::PCLPointCloud2::Ptr cloud)
{   
    sensor_msgs::PointCloud2 output;
    
    pcl_conversions::fromPCL(*cloud, output);

    this->pub_filtered.publish(output);
}

void CloudFilter::publishClustered(pcl::PCLPointCloud2::Ptr cloud, const std::string &frame_id)
{
    sensor_msgs::PointCloud2 output;
    
    pcl_conversions::fromPCL(*cloud, output);

    output.header.frame_id = frame_id;
    this->pub_clustered.publish(output);
}