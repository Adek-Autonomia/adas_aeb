#include "pipeline.h"
#include "setup_port.h"
#include "tokenizer.h"
#include "tof_node.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "distance_measurement");
    ros::NodeHandle nh;
    Tof_node tn = Tof_node(&nh);
    std::cout << "jebać pw i dziekana";
    
    while(ros::ok()) {

        tn.publish_dist();
        ros::spinOnce();
    }
    
}