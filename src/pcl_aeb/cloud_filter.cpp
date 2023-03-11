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

    this->setROI();

    this->handle.getParam("merged_pcl_topic", paramTmp);
    this->sub = handle.subscribe("/cloud_merged", 10, &CloudFilter::callback, this); //topic hardcoded, error with launch param
    
    this->handle.getParam("stop_topic", paramTmp);
    this->pub_stopFlag = this->handle.advertise<std_msgs::Bool>(paramTmp, 1, false);

    // this->pub_filtered = this->handle.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1, false);
    this->pub_clustered = this->handle.advertise<sensor_msgs::PointCloud2>("/cloud_clustered", 1, false);
    // this->pub_segmented = this->handle.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1, false);
    this->pub_bbox = this->handle.advertise<vision_msgs::Detection3DArray>("/pointCloud/detections", 1, false);

}

void CloudFilter::setROI()
{
    float x1 = 0.4;
    float x2 = 3.0;
    float y = 0.3;
    this->ROI.reserve(4);
    this->ROI[0] = cv::Point2f(x1, -y);
    this->ROI[1] = cv::Point2f(x2, -y);
    this->ROI[2] = cv::Point2f(x2, y);
    this->ROI[3] = cv::Point2f(x1, y);
}

void CloudFilter::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    ROS_DEBUG("cloud recieved, # of points: %lu", cloud_msg->data.size());

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    std::vector<Bbox> boxes;
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

    boxes.reserve(clusters.size());
    for (size_t i = 0; i<clusters.size(); i++)
    {
        Bbox box;
        this->findBoundingBox(clusters[i], box);

        boxes.push_back(box);
    }

    output_msg.detections.reserve(boxes.size());
    for (size_t i =0; i<boxes.size(); i++)
    {   
        if(this->isBoxInROI(boxes[i]))
        {
            vision_msgs::BoundingBox3D box = this->toRosBBox(boxes[i]);
            vision_msgs::Detection3D det;
            det.bbox = box;
            output_msg.detections.push_back(det);   
        }
    }

    pcl::toPCLPointCloud2(*clusteredCloud, *cloud);

    this->publishClustered(cloud, cloud_msg->header.frame_id);

    this->pub_bbox.publish(output_msg);

    ROS_DEBUG("number of clusters: %lu", clusters.size());
    ROS_DEBUG("number of boxes in ROI: %lu", boxes.size());
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
    //old approach, not used
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

void CloudFilter::findBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Bbox &box)
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
    cv::RotatedRect rect = cv::minAreaRect(points);
    box.XYPlane = rect;
    box.zBound = minMaxZ;
}

bool CloudFilter:: isBoxInROI(Bbox& box)
{
    bool test_output = false;
    std::vector<cv::Point2f> vertices;
    vertices.reserve(4);
    cv::boxPoints(box.XYPlane, vertices);
    for(size_t i = 0; i<vertices.size(); i++)
    {
        test_output += this->isPointInROI(vertices[i]);
    }
    return test_output;
}

bool CloudFilter:: isPointInROI(cv::Point2f &point)
{
    float ABParea = this->areaOfTriangle(this->ROI[0], this->ROI[1], point);
    float BCParea = this->areaOfTriangle(this->ROI[1], this->ROI[2], point);
    float CDParea = this->areaOfTriangle(this->ROI[2], this->ROI[3], point);
    float DAParea = this->areaOfTriangle(this->ROI[3], this->ROI[0], point);

    float ROIarea = std::abs((this->ROI[0].x - this->ROI[2].x)*(this->ROI[0].y - this->ROI[2].y));

    return (ABParea + BCParea + CDParea + DAParea) > ROIarea;
}

float CloudFilter:: areaOfTriangle(cv::Point2f &A, cv::Point2f &B, cv::Point2f &C)
{
    float area = std::abs(A.x*(B.y-C.y) + B.x*(C.y-A.y) + C.x*(A.y - B.y))/2;
    return area;
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