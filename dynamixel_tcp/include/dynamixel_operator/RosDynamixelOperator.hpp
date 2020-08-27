#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "dynamixel_operator/dynamixel_operator.hpp"

namespace dynamixel_tcp
{
    enum ShakeHeadState
    {
        LEFT,
        RIGHT
    }

    class RosDynamixelOperator
    {
    public:
        RosDynamixelOperator(ros::NodeHandle &nodeHandle);
        virtual ~RosDynamixelOperator();

    private:
        bool isTargetPositionArrived() const;
        bool shakeHeadServiceCallback(std_srvs::SetBool::Request &request,
                                      std_srvs::SetBool::Response &response);

        void moveToHome();

        static int const TOLERANCE = 10;

        //Actually head_shaking can be interface
        bool head_shaking;
        ShakeHeadState shake_head_state;
        ros::Service shakehead_srv;
        uint16_t shake_head_cw_lim;
        uint16_t shake_head_ccw_lim;

        ros::NodeHandle &nh;
        ros::Publisher target_publisher;
        ros::Subscriber current_subscriber;

        sensor_msgs::JointState target_state;
        sensor_msgs::JointState current_state;
        sensor_msgs::JointState home_state;
    };

} // namespace dynamixel_tcp