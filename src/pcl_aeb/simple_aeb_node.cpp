#include "adas_aeb/pcl_aeb/cloud_filter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_aeb_node");

    ros::NodeHandle nh, pnh("~");

    CloudFilter cloudFilter(nh, pnh);

    ros::spin();

    return 0;
}

//TO DO
//
//creating a "space of interest" -> adjusting the box
//populating the SOI with the points from PCL that are within
// make a decision based on the points in the output cloud