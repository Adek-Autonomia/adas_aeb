#include "adas_aeb/pcl_aeb/cloud_filter.h"

CloudFilter::CloudFilter(adas::ConfigParser& parser, const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : handle(nh), privateHandle(pnh)
{
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    std::string sParamTmp;
    float fParamTmp;

    this->leafSize = parser.getFloat("leaf_size");
    ROS_DEBUG("%f", this->leafSize);

    this->setROI();

    this->handle.getParam("merged_pcl_topic", sParamTmp);
    this->sub = handle.subscribe("/cloud_merged", 10, &CloudFilter::callback, this); //topic hardcoded, error with launch param
    
    this->handle.getParam("stop_topic", sParamTmp);
    this->pub_stopFlag = this->handle.advertise<std_msgs::Bool>(sParamTmp, 1, false);

    this->pub_clustered = this->handle.advertise<sensor_msgs::PointCloud2>("/cloud_clustered", 1, false);

    this->pub_bbox = this->handle.advertise<vision_msgs::Detection3DArray>("/pointCloud/detections", 1, false);
    this->pub_markers = this->handle.advertise<visualization_msgs::MarkerArray>("/pointCloud/Boxes", 1, false);

}

void CloudFilter::setROI()
{
    // do cfg
    float x1 = 0.4;
    float x2 = 3.0;
    float y = 0.15;
    this->ROI.reserve(4);
    this->ROI[0] = cv::Point2f(x1, -y);
    this->ROI[1] = cv::Point2f(x2, -y);
    this->ROI[2] = cv::Point2f(x2, y);
    this->ROI[3] = cv::Point2f(x1, y);
}

void CloudFilter::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_DEBUG("cloud recieved, # of points: %lu", cloud_msg->data.size());

    pcl::PointCloud<pcl::PointXYZ>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    vision_msgs::Detection3DArray output_msg;
    visualization_msgs::MarkerArray vis_markers;

    this->getClusters(*cloud_msg, clusters, clusteredCloud);

    output_msg.detections.reserve(clusters.size());
    vis_markers.markers.reserve(clusters.size());

    for (size_t i = 0; i<clusters.size(); i++)
    {
        Bbox temp_box;
        visualization_msgs::Marker temp_marker;
        this->findBoundingBox(clusters[i], temp_box);
        bool inROI = this->isBoxInROI(temp_box);
        this->createMarker(temp_marker, inROI, i, temp_box);
        temp_marker.header.frame_id = cloud_msg->header.frame_id;
        vis_markers.markers.push_back(temp_marker);
        
        if (inROI)
        {
            vision_msgs::BoundingBox3D bbox = this->toRosBBox(temp_box);
            vision_msgs::Detection3D det;
            det.bbox = bbox;
            output_msg.detections.push_back(det);
        }
    }

    this->publishClustered(clusteredCloud, cloud_msg->header.frame_id);

    this->pub_bbox.publish(output_msg);
    this->pub_markers.publish(vis_markers);

    ROS_DEBUG("number of clusters: %lu", clusters.size());
    ROS_DEBUG("number of boxes in ROI: %lu", output_msg.detections.size());
}

void CloudFilter::getClusters(const sensor_msgs::PointCloud2 &cloud_msg, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr rgbCloud)
{
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);

    pcl_conversions::toPCL(cloud_msg, *cloud);

    this->downSample(cloud);

    //conversion needed for further filtering
    pcl::fromPCLPointCloud2(*cloud, *rgbCloud);

    // filtering out the street by z coordinate
    this->stripStreetArea(rgbCloud);

    ROS_DEBUG("# of points after voxelizing: %lu", rgbCloud->size());

    this->clustering(rgbCloud, clusters);

}

void CloudFilter::downSample(pcl::PCLPointCloud2::Ptr PCLcloud)
{
    pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
    vg.setLeafSize(this->leafSize, this->leafSize, this->leafSize);
    // float ls = this->
    // vg.setLeafSize(0.01, 0.01, 0.01);
    vg.setInputCloud(PCLcloud);
    vg.filter(*PCLcloud);
}

void CloudFilter::stripStreetArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.02,FLT_MAX);
    pass.filter(*cloud);
}

void CloudFilter::clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters)
{
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
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

void CloudFilter::publishClustered(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string &frame_id)
{
    pcl::PCLPointCloud2::Ptr cloud_pcl (new pcl::PCLPointCloud2);
    sensor_msgs::PointCloud2 output;

    pcl::toPCLPointCloud2(*cloud, *cloud_pcl);

    pcl_conversions::fromPCL(*cloud_pcl, output);

    output.header.frame_id = frame_id;

    this->pub_clustered.publish(output);
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

vision_msgs::BoundingBox3D& CloudFilter:: toRosBBox(Bbox &input_box)
{
    static vision_msgs::BoundingBox3D output;

    output.center.orientation.w = input_box.XYPlane.angle;
    output.center.orientation.x = output.center.orientation.y = 0;
    output.center.orientation.z = 1;
    
    output.center.position.x = input_box.XYPlane.center.x;
    output.center.position.y = input_box.XYPlane.center.y;
    output.center.position.z = (input_box.zBound.x + input_box.zBound.y)/2;

    output.size.x = input_box.XYPlane.size.width;
    output.size.y = input_box.XYPlane.size.height;
    output.size.z = input_box.zBound.y - input_box.zBound.x;

    return output;
}

void CloudFilter::createMarker(visualization_msgs::Marker &marker, bool inRoi, unsigned long id, Bbox &box)
{
    marker.ns = "vis";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = box.XYPlane.center.x;
    marker.pose.position.y = box.XYPlane.center.y;
    marker.pose.position.z = (box.zBound.y + box.zBound.x)/2;

    marker.pose.orientation.x = marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 1;
    marker.pose.orientation.w = box.XYPlane.angle;

    marker.scale.x = box.XYPlane.size.width;
    marker.scale.y = box.XYPlane.size.height;
    marker.scale.z = box.zBound.y - box.zBound.x;

    marker.color.r = 1.0f*(!inRoi);
    marker.color.g = 1.0f*inRoi;
    marker.color.b = 0.0f;
    marker.color.a = 0.3f;

}
