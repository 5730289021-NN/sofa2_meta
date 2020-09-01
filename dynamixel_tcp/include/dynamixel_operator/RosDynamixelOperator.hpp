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
        std::vector<ros::ServiceServer> services;

        bool readParameters();
        std::vector<std::string> operator_names;
        std::vector<std::string> operator_types;
        std::map<std::string, Operator*> operator_map;

        bool initializeServices(ros::Publisher& target_state_publisher);
    };

} // namespace dynamixel_tcp