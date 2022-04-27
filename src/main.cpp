#include "include/pipeline.h"
#include "include/setup_port.h"
#include "include/tokenizer.h"
#include "include/tof_node.h"

int main(int argc, char **argv) {
/*
    Setup my_port;
    my_port.SetEverything();


    std::string data = my_port.ReadInput();
    std::vector<std::string> first_try;
    std::vector<std::vector<float>> second_try;


    first_try = Tokenizer::tokenize(data, "\n");

    Pipeline pipeline(first_try);
    
    second_try = pipeline.ProcessData();
*/
    ros::init(argc, argv, "distance_measurement");
    ros::NodeHandle nh;
    Tof_node tn = Tof_node(&nh);
    ros::spin();
}