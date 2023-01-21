
#include <ros/ros.h>
#include "ptu_control/PTUControl.hpp"

namespace ptu_control
{
    PTUControl::PTUControl(ros::NodeHandle nodeHandle): nodeHandle_(nodeHandle), seq_(0)
    {

        if(!loadParam())
        {
            ROS_ERROR("Could not load parameters");
            ros::shutdown();
        }

        panTiltServer_ = nodeHandle_.advertiseService(serviceName_, &PTUControl::panTiltCallback, this);
        ROS_INFO("Service Ready");

        jointStateSubscriber_ = nodeHandle_.subscribe("joint_states", 1, &PTUControl::subscriberCallback, this);

        panPositionPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("pan_position", 1, true);
        tiltPositionPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("tilt_position", 1, true);
        panVelocityPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("pan_speed", 1, true);
        tiltVelocityPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("tilt_speed", 1, true);

    }

    PTUControl::~PTUControl(){}

    bool PTUControl::loadParam()
    {
        bool success = true;
        success &= nodeHandle_.getParam("/ptu_control_node/service_name", serviceName_);
        success &= nodeHandle_.getParam("/ptu_control_node/position", jointState_.position);
        success &= nodeHandle_.getParam("/ptu_control_node/velocity", jointState_.velocity);
        success &= nodeHandle_.getParam("/ptu_control_node/effort", jointState_.effort);

        jointState_.name = {"ptu_pan", "ptu_tilt"}; 

        jointState_.header.frame_id = "";
        jointState_.header.seq = seq_++;
        jointState_.header.stamp = ros::Time::now();

        return success;
    }

    bool PTUControl::panTiltCallback(ptu_control::pan_tilt::Request &req, ptu_control::pan_tilt::Response &res)
    {
        std_msgs::Float64 pan;
        pan.data = -1 * req.pan;
        std_msgs::Float64 tilt;
        tilt.data = -1 * req.tilt;

        // ROS_INFO("Moving to %0.2f %0.2f", pan.data, tilt.data);
        panPositionPublisher_.publish(pan);
        tiltPositionPublisher_.publish(tilt);
    
        res.success = true;
        return true;
    }

    void PTUControl::subscriberCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        ROS_DEBUG("Pan: %0.3f, Tilt: %0.3f", msg->position[0], msg->position[1]);

    }


}