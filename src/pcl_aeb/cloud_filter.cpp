#include "adas_aeb/pcl_aeb/cloud_filter.h"

CloudFilter::CloudFilter(float leafSize, float ROI_min_x, float ROI_max_x, float ROI_y, float street_level, float cluster_tolerance, int min_cluster_size, int max_cluster_size)
    : leafSize(leafSize), ROI_min_x(ROI_min_x), ROI_max_x(ROI_max_x), ROI_y(ROI_y), street_level(street_level), cluster_tolerance(cluster_tolerance), min_cluster_size(min_cluster_size), max_cluster_size(max_cluster_size)
{
    this->setROI();
}

void CloudFilter::setROI()
{
    this->ROI.reserve(4);
    this->ROI[0] = cv::Point2f(this->ROI_min_x, -this->ROI_y);
    this->ROI[1] = cv::Point2f(this->ROI_max_x, -this->ROI_y);
    this->ROI[2] = cv::Point2f(this->ROI_max_x, this->ROI_y);
    this->ROI[3] = cv::Point2f(this->ROI_min_x, this->ROI_y);
}



void CloudFilter::getClusters(pcl::PCLPointCloud2::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr rgbCloud, std::vector<Bbox>&boxes, std::vector<bool> &inROI)
{
    this->downSample(cloud);

    //conversion needed for further filtering
    pcl::fromPCLPointCloud2(*cloud, *rgbCloud);

    // filtering out the street by z coordinate
    this->stripStreetArea(rgbCloud);

    this->clustering(rgbCloud, clusters);

    for (size_t i= 0; i<clusters.size(); i++)
    {
        Bbox temp_box;
        this->findBoundingBox(clusters[i], temp_box);
        boxes.push_back(temp_box);
        inROI.push_back(this->isBoxInROI(temp_box));
    }
    
}

void CloudFilter::downSample(pcl::PCLPointCloud2::Ptr PCLcloud)
{
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setLeafSize(this->leafSize, this->leafSize, this->leafSize);
    vg.setInputCloud(PCLcloud);
    vg.filter(*PCLcloud);
}

void CloudFilter::stripStreetArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(this->street_level,FLT_MAX);
    pass.filter(*cloud);
}

void CloudFilter::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(this->cluster_tolerance);
    ec.setMinClusterSize(this->min_cluster_size);
    ec.setMaxClusterSize(this->max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    int counter = 0;
    
    for(auto it=cluster_indices.begin(); it!= cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ>);
        
        for(auto clust_it = it->indices.begin(); clust_it!=it->indices.end(); ++clust_it)
        {
            temp->points.push_back(cloud->points[*clust_it]);
        }
        clusters.push_back(temp); 
        counter ++;
    }
}

void CloudFilter::findBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Bbox &box)
{
    std::vector<cv::Point2f> points;
    points.reserve(cloud->size());
    cv::Point2f minMaxZ(0,0);
    for(auto& p: *cloud )
    {
        if(minMaxZ.x > p.z)//min
        {
            minMaxZ.x = p.z;
        }
        if(minMaxZ.y < p.z)
        {
            minMaxZ.y = p.z;//max
        }
        points.push_back(cv::Point2f(p.x, p.y));
    }
    box.XYPlane = cv::minAreaRect(points);
    box.zBound = minMaxZ;
}

bool CloudFilter:: isBoxInROI(Bbox& box)
{
    cv::RotatedRect roi(this->ROI[0], this->ROI[1], this->ROI[2]);
    std::vector<cv::Point2f> outputArea;
    int overlap = cv::rotatedRectangleIntersection(roi, box.XYPlane, outputArea);
    return overlap != cv::INTERSECT_NONE;
}




