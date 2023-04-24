#include "adas_aeb/pcl_aeb/cloud_filter_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_aeb_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string cfgFilename;
    pnh.getParam("cloud_filter_config_file", cfgFilename);
    adas::ConfigParser parser(cfgFilename);

    CloudFilterNode cloudFilter(parser, nh, pnh);

    ros::spin();

    return 0;
}