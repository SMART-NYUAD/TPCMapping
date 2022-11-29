
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

        // modifiedJointStatePublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("modified_joint_states", 1);

        // modifiedJointStateSubscriber_ = nodeHandle_.subscribe("modified_joint_states", 0, &PTUControl::subscriberCallback, this);

        jointStatePublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("joint_states", 0);
        ros::Duration(0.5).sleep();
        jointStatePublisher_.publish(jointState_);
    }

    PTUControl::~PTUControl(){}

    bool PTUControl::loadParam()
    {
        bool success = true;
        success &= nodeHandle_.getParam("/ptu_control_node/service_name", serviceName_);
        success &= nodeHandle_.getParam("/ptu_control_node/position", jointState_.position);
        success &= nodeHandle_.getParam("/ptu_control_node/velocity", jointState_.velocity);
        success &= nodeHandle_.getParam("/ptu_control_node/effort", jointState_.velocity);

        jointState_.name = {"ptu_pan", "ptu_tilt"}; 

        jointState_.header.frame_id = "";
        jointState_.header.seq = seq_++;
        jointState_.header.stamp = ros::Time::now();

        return success;
    }

    bool PTUControl::panTiltCallback(ptu_control::pan_tilt::Request &req, ptu_control::pan_tilt::Response &res)
    {
        jointState_.header.seq = seq_++;
        jointState_.header.stamp = ros::Time::now();

        jointState_.position = req.position;
        jointState_.velocity = req.velocity;
        jointState_.effort = req.effort;

        ROS_INFO("Moving to %0.2f %0.2f", jointState_.position[0], jointState_.position[1]);
        
        jointStatePublisher_.publish(jointState_);

        res.success = true;
        return true;
    }

    void PTUControl::subscriberCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        // jointStatePublisher_.publish(jointState_);

        // ROS_INFO_STREAM(ros::Time::now());
    }


}