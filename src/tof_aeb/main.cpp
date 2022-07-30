#include "adas_aeb/tof_aeb/"
#include "adas_aeb/tof_aeb/setup_port.h"
#include "adas_aeb/tof_aeb/tokenizer.h"
#include "adas_aeb/tof_aeb/tof_node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "AEB_TOF_Node");
    ros::NodeHandle nh;
    Tof_node tn = Tof_node(&nh);
    
    while(ros::ok()) {

        tn.publish_dist();
        ros::spinOnce();
    }
    
}