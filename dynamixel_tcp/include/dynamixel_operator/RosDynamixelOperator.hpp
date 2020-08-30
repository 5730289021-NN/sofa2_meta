#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include "dynamixel_operator/Operators.hpp"

namespace dynamixel_tcp
{

    class RosDynamixelOperator
    {
    public:
        RosDynamixelOperator(ros::NodeHandle &nodeHandle);
        virtual ~RosDynamixelOperator();
    private:
        ros::NodeHandle &nh;
        ros::Publisher target_state_publisher;

        bool readParameters();
        std::vector<std::string> operator_names;
        std::vector<std::string> operator_types;
        std::map<std::string, Operator*> operator_map;

        bool initializeServices(ros::Publisher& target_state_publisher);

    // private:
    //     bool isTargetPositionArrived() const;
    //     bool shakeHeadServiceCallback(std_srvs::SetBool::Request &request,
    //                                   std_srvs::SetBool::Response &response);

    //     void moveToHome();

    //     static int const TOLERANCE = 10;

    //     //Actually head_shaking can be interface
    //     bool head_shaking;
    //     ShakeHeadState shake_head_state;
    //     ros::Service shakehead_srv;
    //     uint16_t shake_head_cw_lim;
    //     uint16_t shake_head_ccw_lim;

    //     ros::NodeHandle &nh;
    //     ros::Publisher target_publisher;
    //     ros::Subscriber current_subscriber;

    //     sensor_msgs::JointState target_state;
    //     sensor_msgs::JointState current_state;
    //     sensor_msgs::JointState home_state;
    };

} // namespace dynamixel_tcp