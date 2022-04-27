#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Float32.h

#pragma once

class Tof_node {

    private:
        ros::Publisher pub;
        ros::Subscriber dist_subscriber;

    public:
        Tof_node(ros::NodeHandle);
        void dist_callback(const std_msgs::Float32& )


};