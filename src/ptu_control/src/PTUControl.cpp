
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

        stateSubscriber_ = nodeHandle_.subscribe("flir_ptu_ethernet/status", 1, &PTUControl::subscriberCallback, this);

        panPositionPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("pan_position", 1, true);
        tiltPositionPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("tilt_position", 1, true);
        panVelocityPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("pan_speed", 1, true);
        tiltVelocityPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("tilt_speed", 1, true);

        // jointStatePublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("")

        panSpeedSetter_ = nodeHandle_.serviceClient<robotnik_msgs::set_float_value>("/flir_ptu_ethernet/set_max_pan_speed");
        tiltSpeedSetter_ = nodeHandle_.serviceClient<robotnik_msgs::set_float_value>("/flir_ptu_ethernet/set_max_tilt_speed");

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
        robotnik_msgs::set_float_value maxPanSpeed;
        maxPanSpeed.request.value = req.max_pan_speed;

        robotnik_msgs::set_float_value maxTiltSpeed;
        maxTiltSpeed.request.value = req.max_tilt_speed;

        bool success = true;

        success &= panSpeedSetter_.call(maxPanSpeed);
        success &= tiltSpeedSetter_.call(maxTiltSpeed);

        if (!success)
        {
            ROS_ERROR("Failed to set speeds.");
            return false;
        }

        std_msgs::Float64 pan;
        pan.data = -1 * req.pan * M_PI/360;
        std_msgs::Float64 tilt;
        tilt.data = -1 * req.tilt * M_PI/180;

        // ROS_INFO("Moving to %0.2f %0.2f", pan.data, tilt.data);
        panPositionPublisher_.publish(pan);
        tiltPositionPublisher_.publish(tilt);
    
        res.success = true;
        return true;
    }

    void PTUControl::subscriberCallback(const robotnik_msgs::PantiltStatus::ConstPtr &msg)
    {
        // ROS_DEBUG("Pan: %0.3f, Tilt: %0.3f", msg->position[0], msg->position[1]);
        ROS_DEBUG("\nPan: %0.2f, Tilt: %0.2f\nPan Speed: %0.2f, Tilt Speed: %0.2f\n", msg->pan_pos, msg->tilt_pos, msg->pan_speed, msg->tilt_speed);

        // double maxPanSpeed;

        // nodeHandle_.getParam("/flir_ptu_ethernet/max_pan_speed", maxPanSpeed);
        
        // ROS_DEBUG_STREAM(maxPanSpeed);

    }


}