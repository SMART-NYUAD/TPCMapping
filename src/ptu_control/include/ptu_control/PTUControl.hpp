#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <robotnik_msgs/PantiltStatus.h>
#include <robotnik_msgs/set_float_value.h>

#include <vector>
#include <string.h>
#include <math.h>

#include "ptu_control/pan_tilt.h"

namespace ptu_control
{
    class PTUControl
    {
        public:
        PTUControl(ros::NodeHandle nodeHandle);

        ~PTUControl();
        
        private:
        ros::NodeHandle nodeHandle_;
        ros::ServiceServer panTiltServer_;

        ros::Subscriber stateSubscriber_;

        ros::Publisher panPositionPublisher_;
        ros::Publisher tiltPositionPublisher_;
        ros::Publisher panVelocityPublisher_;
        ros::Publisher tiltVelocityPublisher_;

        ros::ServiceClient panSpeedSetter_;
        ros::ServiceClient tiltSpeedSetter_;

        std::string serviceName_;
        sensor_msgs::JointState jointState_;
        ros::Publisher jointStatePublisher_;
        int seq_;
        
        bool loadParam();

        bool panTiltCallback(ptu_control::pan_tilt::Request &req, ptu_control::pan_tilt::Response &res);

        // void subscriberCallback(const sensor_msgs::JointState::ConstPtr &msg);

        void subscriberCallback(const robotnik_msgs::PantiltStatus::ConstPtr &msg);


    };
}