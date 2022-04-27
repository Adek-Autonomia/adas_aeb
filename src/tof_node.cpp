#include "tof_node.h"
#include "include/pipeline.h"
#include "include/setup_port.h"
#include "include/tokenizer.h"

Tof_node::Tof_node(ros::NodeHandle *nh)
{
    pub = nh->advertise<std_msgs::Float32>("/distance_f", 10);
    dist_sub = nh->subscribe("/distance", 1000, &Tof_node::dist_callback, this);

}

void Tof_node::dist_callback(const std_msgs::Float32& msg) {
    Setup my_port;
    my_port.SetEverything();


    std::string data = my_port.ReadInput();
    std::vector<std::string> first_try;
    std::vector<std::vector<float>> second_try;


    first_try = Tokenizer::tokenize(data, "\n");

    Pipeline pipeline(first_try);
    
    second_try = pipeline.ProcessData();
    
    std_msgs::Float32 new_msg;
    new_msg.data = second_try[0][1];
    pub.publish(new_msg);
}