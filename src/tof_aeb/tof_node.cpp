#include "adas_aeb/tof_aeb/tof_node.h"
#include "adas_aeb/tof_aeb/pipeline.h"
#include "adas_aeb/tof_aeb/setup_port.h"
#include "adas_aeb/tof_aeb/tokenizer.h"

/**
 * @brief Initializes new ros node rosponsible for sending distance data from ToF sensors
 * 
 * @param nh ros::NodeHandle object used as a parent element
 */
Tof_node::Tof_node(ros::NodeHandle *nh, ros::NodeHandle* pnh)
{
    std::string paramTmp;
    pnh->getParam("~stop_topic", paramTmp);

    this->pub = nh->advertise<std_msgs::Float32>(paramTmp, 10);

    this->my_port.SetEverything();
}

/**
 * @brief Publishes gathered data
 * 
 */
void Tof_node::publish_dist(){
    std::string data = my_port.ReadInput();
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