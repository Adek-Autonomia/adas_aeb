#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float32.h>
#include "adas_aeb/tof_aeb/setup_port.h"
#include "adas_aeb/tof_aeb/pipeline.h"

#pragma once

class Tof_node {

    private:
        ros::Publisher pub;
        Setup my_port;
        Pipeline pipeline;

    public:
        Tof_node(ros::NodeHandle*, ros::NodeHandle*);
        void dist_callback(const std_msgs::Float32&);
        void publish_dist();


};