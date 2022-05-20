#include "tof_node.h"
#include "pipeline.h"
#include "setup_port.h"
#include "tokenizer.h"

Tof_node::Tof_node(ros::NodeHandle *nh)
{
    pub = nh->advertise<std_msgs::Float32>("/distance", 10);
    my_port.SetEverything();

}
/*
void Tof_node::dist_callback(const std_msgs::Float32& msg) {
    
    std::string data = my_port.ReadInput();
    std::vector<std::string> first_try;
    std::vector<std::vector<float>> second_try;


    first_try = Tokenizer::tokenize(&data, "\n");

    Pipeline pipeline(first_try);
    
    second_try = pipeline.ProcessData();
    
    std_msgs::Float32 new_msg;
    new_msg.data = second_try[0][1];
    pub.publish(new_msg);
}
*/
void Tof_node::publish_dist(){
    std::string data = my_port.ReadInput();
    std::cout << "Input size: " << data.size() << "\n";
    if (data.empty() == false){
        std::vector<std::string> first_try;
        std::vector<std::vector<float>> second_try;


        first_try = Tokenizer::tokenize(&data, "\n");

        pipeline.set_input(first_try);

        second_try = pipeline.ProcessData();

        std_msgs::Float32 new_msg;
        new_msg.data = second_try[0][1];
        pub.publish(new_msg);
    }
}