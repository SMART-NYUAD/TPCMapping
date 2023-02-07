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

        sensor_msgs::JointState jointState_;
        ros::Publisher jointStatePublisher_;
        ros::Subscriber jointStateSubscriber_;

        ros::Publisher statusPublisher_;

        ros::Publisher panPositionPublisher_;
        ros::Publisher tiltPositionPublisher_;
        ros::Publisher panVelocityPublisher_;
        ros::Publisher tiltVelocityPublisher_;

        ros::ServiceClient panSpeedSetter_;
        ros::ServiceClient tiltSpeedSetter_;

        std::string serviceName_;

        int seq_;

        double panMin_;
        double panMax_;
        double tiltMin_;
        double tiltMax_;
        double panSpeedLimit_;
        double tiltSpeedLimit_;
        double panCalibration_;
        double tiltCalibration_;
        
        bool loadParam();

        bool panTiltCallback(ptu_control::pan_tilt::Request &req, ptu_control::pan_tilt::Response &res);

        void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

    };
}