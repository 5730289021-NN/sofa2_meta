#pragma once
#include <ros/ros.h>
#include <vector>
#include <string>
#include <base_operator/Operation.hpp>

namespace base_operator
{
    class RosBaseOperator
    {
    public:
        RosBaseOperator(ros::NodeHandle &nodeHandle);
        ~RosBaseOperator();

    private:
        ros::NodeHandle &nh;
        ros::Publisher publisher;
        std::vector<ros::ServiceServer> services;

        bool readParameters();
        std::vector<std::string> operator_names;
        std::vector<std::string> operator_types;
        std::map<std::string, Operation *> operator_map;

        bool initializeServices(ros::Publisher &publisher);
    };
} // namespace base_operator