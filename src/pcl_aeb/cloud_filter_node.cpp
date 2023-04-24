#include "adas_aeb/pcl_aeb/cloud_filter_node.h"

CloudFilterNode::CloudFilterNode(adas::ConfigParser& parser, const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
    : handle(nh), privateHandle(pnh)
{
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    std::string sParamTmp;

    float leafSize = parser.getFloat("leaf_size");
    float ROI_min_x = parser.getFloat("FOV_min_dist");
    float ROI_max_x = parser.getFloat("FOV_max_dist");
    float ROI_y = parser.getFloat("FOV_side_half_dist");
    float street_level = parser.getFloat("Street_level");
    float cluster_tolerance = parser.getFloat("cluster_tolerance");
    int min_cluster_size = parser.getInt("min_cluster_size");
    int max_cluster_size = parser.getInt("max_cluster_size");

    this->cloudFilter = CloudFilter(leafSize, ROI_min_x, ROI_max_x, ROI_y, street_level, cluster_tolerance, min_cluster_size, max_cluster_size);

    this->privateHandle.getParam("merged_pcl_topic", sParamTmp);
    this->sub = handle.subscribe(sParamTmp, 10, &CloudFilterNode::callback, this);
    
    this->privateHandle.getParam("stop_topic", sParamTmp);
    this->pub_stopFlag = this->handle.advertise<std_msgs::Bool>(sParamTmp, 1, false);

    this->privateHandle.getParam("cloud_clustered_topic", sParamTmp);
    this->pub_clustered = this->handle.advertise<sensor_msgs::PointCloud2>(sParamTmp, 1, false);

    this->privateHandle.getParam("detections_topic", sParamTmp);
    this->pub_bbox = this->handle.advertise<vision_msgs::Detection3DArray>(sParamTmp, 1, false);

    this->privateHandle.getParam("boxes_topic", sParamTmp);
    this->pub_markers = this->handle.advertise<visualization_msgs::MarkerArray>(sParamTmp, 1, false);

}

void CloudFilterNode::callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //recieve cloud
    //cast to pcl type
    //use <logic> callback
    //publish output

    ROS_DEBUG("cloud recieved, # of points: %lu", cloud_msg->data.size());

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    std::vector<Bbox> boxes;
    std::vector<bool> inROI;

    vision_msgs::Detection3DArray output_msg;
    visualization_msgs::MarkerArray vis_markers;

    clusters.reserve(100);
    this->cloudFilter.getClusters(cloud, clusters, clusteredCloud, boxes, inROI);

    output_msg.detections.reserve(clusters.size());
    vis_markers.markers.reserve(clusters.size());

    for (size_t i = 0; i<clusters.size(); i++)
    {
        visualization_msgs::Marker temp_marker;

        this->createMarker(temp_marker, inROI[i], i, boxes[i]);
        temp_marker.header.frame_id = cloud_msg->header.frame_id;
        vis_markers.markers.push_back(temp_marker);
        
        if (inROI[i])
        {
            vision_msgs::BoundingBox3D bbox = this->toRosBBox(boxes[i]);
            sensor_msgs::PointCloud2::Ptr pcd;
            pcl::toROSMsg(*clusters[i], *pcd);
            vision_msgs::Detection3D det;
            det.bbox = bbox;
            det.source_cloud = *pcd;
            output_msg.detections.push_back(det);
        }
    }

    this->publishClustered(clusteredCloud, cloud_msg->header.frame_id);

    this->pub_bbox.publish(output_msg);
    this->pub_markers.publish(vis_markers);

    ROS_DEBUG("number of clusters: %lu", clusters.size());
    ROS_DEBUG("number of boxes in ROI: %lu", output_msg.detections.size());
}

vision_msgs::BoundingBox3D& CloudFilterNode:: toRosBBox(Bbox &input_box)
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

void CloudFilterNode::createMarker(visualization_msgs::Marker &marker, bool inRoi, unsigned long id, Bbox &box)
{
    marker.ns = "vis";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = box.XYPlane.center.x;
    marker.pose.position.y = box.XYPlane.center.y;
    marker.pose.position.z = (box.zBound.y + box.zBound.x)/2;

    float norm = std::sqrt(1.0 + pow(box.XYPlane.angle*CV_PI/180.0,2) );

    marker.pose.orientation.x = marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 1.0/norm;
    marker.pose.orientation.w = box.XYPlane.angle*CV_PI/180.0/norm;

    marker.scale.x = box.XYPlane.size.width;
    marker.scale.y = box.XYPlane.size.height;
    marker.scale.z = box.zBound.y - box.zBound.x;

    marker.color.r = 1.0f*(!inRoi);
    marker.color.g = 1.0f*inRoi;
    marker.color.b = 0.0f;
    marker.color.a = 0.3f;

}

void CloudFilterNode::publishClustered(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string &frame_id)
{
    pcl::PCLPointCloud2::Ptr cloud_pcl (new pcl::PCLPointCloud2);
    sensor_msgs::PointCloud2 output;

    pcl::toPCLPointCloud2(*cloud, *cloud_pcl);

    pcl_conversions::fromPCL(*cloud_pcl, output);

    output.header.frame_id = frame_id;

    this->pub_clustered.publish(output);
}