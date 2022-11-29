#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <vector>
#include <string.h>

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
        ros::Publisher modifiedJointStatePublisher_;
        ros::Subscriber modifiedJointStateSubscriber_;
        ros::Publisher jointStatePublisher_;
        ros::ServiceServer panTiltServer_;

        std::string serviceName_;
        sensor_msgs::JointState jointState_;
        int seq_;
        
        bool loadParam();

        bool panTiltCallback(ptu_control::pan_tilt::Request &req, ptu_control::pan_tilt::Response &res);

        void subscriberCallback(const sensor_msgs::JointState::ConstPtr &msg);


    };
}