#include "adas_aeb/pcl_aeb/cloud_filter.h"

CloudFilter::CloudFilter(const ros::NodeHandle& nh)
    : handle(nh)
{
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    srand(time(NULL));

    std::string paramTmp;
    float fParamTmp;

    this->handle.getParam("leaf_size", fParamTmp);
    this->leafSize = fParamTmp;
     // (x,y,z)

    this->handle.getParam("merged_pcl_topic", paramTmp);
    this->sub = handle.subscribe("/cloud_merged", 10, &CloudFilter::callback, this); //topic hardcoded, error with launch param
    
    this->handle.getParam("stop_topic", paramTmp);
    this->pub_stopFlag = this->handle.advertise<std_msgs::Bool>(paramTmp, 1, false);

    // this->pub_filtered = this->handle.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1, false);
    this->pub_clustered = this->handle.advertise<sensor_msgs::PointCloud2>("/cloud_clustered", 1, false);
    // this->pub_segmented = this->handle.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1, false);
    this->pub_bbox = this->handle.advertise<vision_msgs::Detection3DArray>("/pointCloud/detections", 1, false);

}

void CloudFilter::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_DEBUG("cloud recieved, # of points: %d", cloud_msg->data.size());

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    vision_msgs::Detection3DArray output_msg;

    // Convert from ros message to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    // voxelizing recieved data and reducing redundant points
    this->downSample(cloud);
    ROS_DEBUG("# of points after voxelizing: %lu", cloud->data.size());

    //conversion needed for further filtering
    pcl::fromPCLPointCloud2(*cloud, *rgbCloud);

    // filtering out the street by z coordinate
    this->stripStreetArea(rgbCloud);

    this->clustering(rgbCloud, clusters);
    
    this->paintClusters(clusters);

    for(size_t i = 0; i<clusters.size(); i++)
    {
        for(auto& point: *clusters[i] )
        {
            clusteredCloud->points.push_back(point);
        }
    }

    for (size_t i = 0; i<clusters.size(); i++)
    {
        Eigen::Vector3f bboxTransform;
        Eigen::Quaternionf bboxQuaternion;
        pcl::PointXYZRGB minPoint, maxPoint;
        this->findBoundingBox(clusters[i], bboxTransform, bboxQuaternion, minPoint, maxPoint);
        vision_msgs::Detection3D det;
        det.bbox.center.orientation.w = bboxQuaternion.w();
        det.bbox.center.orientation.x = bboxQuaternion.x();
        det.bbox.center.orientation.y = bboxQuaternion.y();
        det.bbox.center.orientation.z = bboxQuaternion.z();

        det.bbox.center.position.x = bboxTransform.x();
        det.bbox.center.position.y = bboxTransform.y();
        det.bbox.center.position.z = bboxTransform.z();

        det.bbox.size.x = maxPoint.x - minPoint.x;
        det.bbox.size.y = maxPoint.y - minPoint.y;
        det.bbox.size.z = maxPoint.z - minPoint.z;

        output_msg.detections.push_back(det);
    }

    pcl::toPCLPointCloud2(*clusteredCloud, *cloud);

    this->publishClustered(cloud, cloud_msg->header.frame_id);

    this->pub_bbox.publish(output_msg);

    ROS_DEBUG("number of clusters: %lu", clusters.size());
}

void CloudFilter::paintClusters(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
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
    // vg.setLeafSize(this->leafSize, this->leafSize, this->leafSize);
    vg.setLeafSize(0.01, 0.01, 0.01);
    vg.setInputCloud(PCLcloud);
    vg.filter(*PCLcloud);
}

void CloudFilter::stripStreetArea(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.02,FLT_MAX);
    pass.filter(*cloud);
}

void CloudFilter::clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters)
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

    int counter = 0;
    
    for(auto it=cluster_indices.begin(); it!= cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        for(auto clust_it = it->indices.begin(); clust_it!=it->indices.end(); ++clust_it)
        {
            temp->points.push_back(cloud->points[*clust_it]);
        }
        clusters.push_back(temp); 
        counter ++;
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

void CloudFilter::findBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Vector3f &bboxTransform, Eigen::Quaternionf &bboxQuaternion, pcl::PointXYZRGB minPoint, pcl::PointXYZRGB maxPoint)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    Eigen:: Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectors = eigen_solver.eigenvectors();
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1)); //necessary for correct signs

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());

    projectionTransform.block<3,3>(0,0) = eigenVectors.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projectedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *projectedCloud, projectionTransform);
    // pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*projectedCloud, minPoint, maxPoint);

    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    const Eigen::Quaternionf quaternion(eigenVectors);
    bboxQuaternion = quaternion;
    bboxTransform = eigenVectors * meanDiagonal + centroid.head<3>();
    

}