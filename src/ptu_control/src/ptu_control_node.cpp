#include <ros/ros.h>
#include "ptu_control/PTUControl.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ptu_control_node");
    ros::NodeHandle nodeHandle;

    ptu_control::PTUControl ptu_control(nodeHandle);
    ros::spin();
    return 0;
}