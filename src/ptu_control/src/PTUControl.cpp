
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

        // jointStatePublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("joint_states", 0);


        // ros::Duration(0.5).sleep();
        // jointStatePublisher_.publish(jointState_);

        panPositionPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("pan_position", 1);
        tiltPositionPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("pan_position", 1);
        panVelocityPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("pan_speed", 1);
        tiltVelocityPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("tilt_speed", 1);

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
        jointState_.header.seq = seq_++;
        jointState_.header.stamp = ros::Time::now();

        // jointState_.position = req.position;
        // jointState_.velocity = req.velocity;
        // jointState_.effort = req.effort;


        jointState_.position[0] = req.pan;
        jointState_.position[1] = req.tilt;
        jointState_.velocity[0] = req.panVelocity;
        jointState_.velocity[1] = req.tiltVelocity;
        jointState_.effort[0] = req.panEffort;
        jointState_.effort[1] = req.tiltEffort;

        ROS_INFO("Moving to %0.2f %0.2f", jointState_.position[0], jointState_.position[1]);
        
        // jointStatePublisher_.publish(jointState_);

        std_msgs::Float64 pan;
        pan.data = req.pan;
        std_msgs::Float64 tilt;
        tilt.data = req.tilt;
        std_msgs::Float64 panVelocity;
        panVelocity.data = req.panVelocity;
        std_msgs::Float64 tiltVelocity;
        tiltVelocity.data = req.tiltVelocity;

        panPositionPublisher_.publish(pan);
        tiltPositionPublisher_.publish(tilt);
        panVelocityPublisher_.publish(panVelocity);
        tiltVelocityPublisher_.publish(tiltVelocity);
        // panPositionPublisher_.publish(jointState_.position[0]);
        // tiltPositionPublisher_.publish(jointState_.position[1]);
        // panSpeedPublisher_.publish(jointState_.velocity[0]);
        // tiltSpeedPublisher_.publish(jointState_.velocity[1]);

        res.success = true;
        return true;
    }

    void PTUControl::subscriberCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        // jointStatePublisher_.publish(jointState_);

        // ROS_INFO_STREAM(ros::Time::now());
    }


}