#include "adas_aeb/pcl_aeb/cloud_filter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_aeb_node");

    ros::NodeHandle nh;

    CloudFilter cloudFilter(nh);

    ros::spin();

    return 0;
}