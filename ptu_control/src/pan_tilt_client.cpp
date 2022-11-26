#include <ros/ros.h>
#include "ptu_control/PTUControl.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pan_tilt_client");

    if(argc < 3)
    {
        ROS_INFO("Usage: pan_tilt_client <pan> <tilt>");
        return 1;
    } else if (argc > 7)
    {
        ROS_INFO("Usage: pan_tilt_client <pan> <tilt> <v1> <v2> <e1> <e>");
        return 1;
    }

    ros::NodeHandle nodeHandle;
    ros::ServiceClient panTiltClient = nodeHandle.serviceClient<ptu_control::pan_tilt>("pan_tilt");
    
    std::vector<double> position {0.0, 0.0};
    std::vector<double> velocity {0.0, 0.0};
    std::vector<double> effort {0.0, 0.0};

    std::vector<double> inputs {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for(int i = 1; i < argc; i++)
    {
        inputs[i-1] = atof(argv[i]);
    }

    position[0] = inputs[0];
    position[1] = inputs[1];
    velocity[0] = inputs[2];
    velocity[1] = inputs[3];
    effort[0] = inputs[4];
    effort[1] = inputs[5];

    ptu_control::pan_tilt service;

    service.request.position = position;
    service.request.velocity = velocity;
    service.request.effort = effort;

    if(panTiltClient.call(service))        
    {
        ROS_INFO("Service Client Called");
    }
    else
    {
        ROS_ERROR("Failed to call service.");
        return 1;
    }

    return 0;
}