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

        jointStateSubscriber_ = nodeHandle_.subscribe("raw_joint_states", 1, &PTUControl::jointStateCallback, this);

        jointStatePublisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("joint_states", 1);

        statusPublisher_ = nodeHandle_.advertise<robotnik_msgs::PantiltStatus>("status", 1);

        panPositionPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("pan_position", 1, true);
        tiltPositionPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("tilt_position", 1, true);

        //The below velocity publishers are unimplemented but available functionality. 
        panVelocityPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("pan_velocity", 1, true);
        tiltVelocityPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("tilt_velocity", 1, true);

        panSpeedSetter_ = nodeHandle_.serviceClient<robotnik_msgs::set_float_value>("/flir_ptu_ethernet/set_max_pan_speed");
        tiltSpeedSetter_ = nodeHandle_.serviceClient<robotnik_msgs::set_float_value>("/flir_ptu_ethernet/set_max_tilt_speed");
        

        ROS_INFO("\nPhysical Limits:\n------------------\n-150 <= Pan <= 150\n-45 <= Tilt <= 30\n0 <= Pan Speed <= 130\n0 <= Tilt Speed <= 30");

        ptu_control::pan_tilt service;
        service.request.pan = jointState_.position[0];
        service.request.tilt = jointState_.position[1];
        service.request.max_pan_speed = jointState_.velocity[0];
        service.request.max_tilt_speed = jointState_.velocity[1];

        ros::Duration(1.5).sleep();

        PTUControl::panTiltCallback(service.request, service.response);

    }

    PTUControl::~PTUControl(){}

    bool PTUControl::loadParam()
    {
        bool success = true;
        success &= nodeHandle_.getParam("/ptu_control_node/service_name", serviceName_);
        success &= nodeHandle_.getParam("/ptu_control_node/position", jointState_.position);
        success &= nodeHandle_.getParam("/ptu_control_node/velocity", jointState_.velocity);
        success &= nodeHandle_.getParam("/ptu_control_node/effort", jointState_.effort);

        success &= nodeHandle_.getParam("/ptu_control_node/pan_min", panMin_);
        success &= nodeHandle_.getParam("/ptu_control_node/pan_max", panMax_);
        success &= nodeHandle_.getParam("/ptu_control_node/tilt_min", tiltMin_);
        success &= nodeHandle_.getParam("/ptu_control_node/tilt_max", tiltMax_);
        success &= nodeHandle_.getParam("/ptu_control_node/pan_speed_limit", panSpeedLimit_);
        success &= nodeHandle_.getParam("/ptu_control_node/tilt_speed_limit", tiltSpeedLimit_);
        success &= nodeHandle_.getParam("/ptu_control_node/pan_calibration", panCalibration_);
        success &= nodeHandle_.getParam("/ptu_control_node/tilt_calibration", tiltCalibration_);

        jointState_.name = {"ptu_pan", "ptu_tilt"}; 

        jointState_.header.frame_id = "";
        jointState_.header.seq = seq_++;
        jointState_.header.stamp = ros::Time::now();

        return success;
    }

    bool PTUControl::panTiltCallback(ptu_control::pan_tilt::Request &req, ptu_control::pan_tilt::Response &res)
    {
        double pan = req.pan;
        double tilt = req.tilt;
        double maxPanSpeed = req.max_pan_speed;
        double maxTiltSpeed = req.max_tilt_speed;

        if (pan < panMin_ || pan > panMax_)
        {
            ROS_WARN("Pan angle out of range.\n");
            ROS_WARN("\nPhysical Limits:\n------------------\n-150 <= Pan <= 150\n-45 <= Tilt <= 30\n0 <= Pan Speed <= 130\n0 <= Tilt Speed <= 30");
            res.success = false;
            return false;
        }
        else if (tilt < tiltMin_ || tilt > tiltMax_)
        {
            ROS_WARN("Tilt angle out of range.\n");
            ROS_WARN("\nPhysical Limits:\n------------------\n-150 <= Pan <= 150\n-45 <= Tilt <= 30\n0 <= Pan Speed <= 130\n0 <= Tilt Speed <= 30");
            res.success = false;
            return false;
        }
        else if (maxPanSpeed < 0 || maxPanSpeed > panSpeedLimit_)
        {
            ROS_WARN("Pan speed out of range.\n");
            ROS_WARN("\nPhysical Limits:\n------------------\n-150 <= Pan <= 150\n-45 <= Tilt <= 30\n0 <= Pan Speed <= 130\n0 <= Tilt Speed <= 30");
            res.success = false;
            return false;
        }
        else if (maxTiltSpeed < 0 || maxTiltSpeed > tiltSpeedLimit_)
        {
            ROS_WARN("Tilt speed limit out of range.\n");
            ROS_WARN("\nPhysical Limits:\n------------------\n-150 <= Pan <= 150\n-45 <= Tilt <= 30\n0 <= Pan Speed <= 130\n0 <= Tilt Speed <= 30");
            res.success = false;
            return false;
        }

        robotnik_msgs::set_float_value maxPanSpeedMsg;
        maxPanSpeedMsg.request.value = maxPanSpeed / 2;

        robotnik_msgs::set_float_value maxTiltSpeedMsg;
        maxTiltSpeedMsg.request.value = maxTiltSpeed;

        bool success = true;

        success &= panSpeedSetter_.call(maxPanSpeedMsg);
        success &= tiltSpeedSetter_.call(maxTiltSpeedMsg);

        if (!success)
        {
            ROS_ERROR("Failed to set speeds.");
            return false;
        }

        std_msgs::Float64 panMsg;
        panMsg.data = -1 * (pan - panCalibration_) * M_PI/360;
        std_msgs::Float64 tiltMsg;
        tiltMsg.data = -1 * (tilt - tiltCalibration_) * M_PI/180;

        // ROS_INFO("Moving to %0.2f %0.2f", pan.data, tilt.data);
        panPositionPublisher_.publish(panMsg);
        tiltPositionPublisher_.publish(tiltMsg);

        ROS_INFO("\nService called\n------------------\nPan: %0.2f, Tilt: %0.2f\nPan Speed; %0.2f, Tilt Speed: %0.2f\n------------------\n", pan, tilt, maxPanSpeed, maxTiltSpeed);
    
        res.success = true;
        return true;
    }

    void PTUControl::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        sensor_msgs::JointState fixedState;

        fixedState = *msg;

        fixedState.name.clear();
        fixedState.name.push_back("ptu_pan");
        fixedState.name.push_back("ptu_tilt");

        fixedState.position[0] = fixedState.position[0] * 2 + panCalibration_ * M_PI/180;
        fixedState.position[1] = fixedState.position[1] + tiltCalibration_ * M_PI/180;

        fixedState.velocity[0] = fixedState.velocity[0] * 2;

        jointStatePublisher_.publish(fixedState);

        double pan = fixedState.position[0] * 180/M_PI;
        double tilt = fixedState.position[1] * 180/M_PI;
        double panSpeed = fixedState.velocity[0] * 180/M_PI;
        double tiltSpeed = fixedState.velocity[1] * 180/M_PI;

        robotnik_msgs::PantiltStatus status;
        status.pan_pos = pan;
        status.tilt_pos = tilt;
        status.pan_speed = panSpeed;
        status.tilt_speed = tiltSpeed;

        // ROS_DEBUG("\nPan: %0.2f, Tilt: %0.2f\nPan Speed; %0.2f, Tilt Speed: %0.2f\n", pan, tilt, panSpeed, tiltSpeed);
        statusPublisher_.publish(status);

    }
}