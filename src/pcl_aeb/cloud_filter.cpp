#include "adas_aeb/pcl_aeb/cloud_filter.h"

CloudFilter::CloudFilter()
{
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
      ros::console::notifyLoggerLevelsChanged();
  }

    sub = node.subscribe("/cloud_merged", 10, &CloudFilter::callback, this);
    pub_marker = node.advertise<visualization_msgs::Marker>("/bounding_box", 1, false);
    pub_stopFlag = node.advertise<std_msgs::Bool>("/bool/aeb/stop", 1, false);


    //setting values for boundary box 
    setOrigin();
    setEdge();
    //creating the boundary box;
    marker = createPointList();
}

void CloudFilter::callback(const sensor_msgs::PointCloud2ConstPtr& cloudIn)
{
    // sensor_msgs::PointCloud2Ptr cloud;
    filteringPoints(cloudIn);
    pub_marker.publish(marker);
    // pub_cloud.publish(*cloud);
}

void CloudFilter::setOrigin()
{
    origin.x = 0.4;
    origin.y = -0.15;
    origin.z = -0.02;
}

void CloudFilter::setEdge()
{
    edge.x = 0.8;
    edge.y = 0.3;
    edge.z = 0.35;
}

visualization_msgs::Marker CloudFilter::createPointList()
{
    visualization_msgs::Marker pointList;
    
    pointList.header.frame_id = "base_link";
    pointList.header.stamp = ros::Time::now();
    pointList.ns = "pointList";
    pointList.id = 1;
    pointList.type = visualization_msgs::Marker::POINTS;
    pointList.action = visualization_msgs::Marker::ADD;

    pointList.pose.orientation.w = 1.0;

    pointList.scale.x = 0.05;
    pointList.scale.y = 0.05;

    pointList.color.g = 1.0f;
    pointList.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = origin.x;
    p.y = origin.y;
    p.z = origin.z;

    //setting vertices of boundary box to filter points from pointcloud
    //looking for an idea how to do it ... cleaner
    pointList.points.push_back(p);
    p.x += edge.x;
    pointList.points.push_back(p);
    p.x = origin.x;
    p.y += edge.y;
    pointList.points.push_back(p);
    p.x += edge.x;
    pointList.points.push_back(p);
    p.x = origin.x;
    p.y = origin.y;
    p.z += edge.z;
    pointList.points.push_back(p);
    p.x += edge.x;
    pointList.points.push_back(p);
    p.x = origin.x;
    p.y += edge.y;
    pointList.points.push_back(p);
    p.x += edge.x;
    pointList.points.push_back(p);

    return pointList;
}

bool CloudFilter::filteringPoints(const sensor_msgs::PointCloud2ConstPtr& cloudIn)
{   
    bool needToStop = false;
    int pointCounter = 0;
    // sensor_msgs::PointCloud2Ptr cloudOut;
    sensor_msgs::PointCloud2ConstIterator<float> it(*cloudIn, "x");

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointIndices:: Ptr indices(new pcl::PointIndices);
    // pcl::ExtractIndices<pcl::PointXYZ> extract;

    // pcl::fromROSMsg(*cloudIn, *cloudPCL);

    //not working...

    // for(int i = 0; i< cloudPCL->points.size(); i++)
    // {
    //     // ROS_DEBUG("%f", cloudPCL.points[i].x);
    //     pcl:: PointXYZ pt = cloudPCL->points[i];
    //     int counter = 0;
    //     counter += (origin.y < pt.y && pt.y < (origin.y + edge.y));
    //     counter += (origin.z < pt.z && pt.z < (origin.z + edge.z));
    //     counter += (origin.x < pt.x && pt.x < (origin.x + edge.x));
    //     if (counter != 3)
    //     {
    //         indices->indices.push_back(i);
    //         ROS_DEBUG("no");
    //     }
    // }
    // extract.setInputCloud(cloudPCL);
    // extract.setIndices(indices);
    // extract.setNegative(true);
    // extract.filter(*cloudPCL);

    // pcl::toROSMsg(*cloudPCL, *cloudOut);

    for(it; it !=it.end(); ++it)
    {
        int counter = 0;

        counter += (origin.x < it[0] && it[0] < (origin.x + edge.x));
        counter += (origin.y < it[1] && it[1] < (origin.y + edge.y));
        counter += (origin.z < it[2] && it[2] < (origin.z + edge.z));

        if (counter == 3)
        {
            // ROS_DEBUG("yes %f, %f, %f", it[0], it[1], it[2]);
            pointCounter++;
        }
        //amount of points is very arbitrary
        // if it's 20000 car will only stop at walls
        // human model is roughly 3000 to 3500 points, cars circa 7500
        if (pointCounter == 2500)
        {   needToStop = true;
            break;
        }
    }
    ROS_DEBUG("%d", pointCounter);

    return needToStop;
}
