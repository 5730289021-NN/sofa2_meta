#pragma once
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include "Operation.hpp"

namespace base_operator
{
    class Action : public Operation
    {
    public:
        Action(ros::Publisher& publisher, geometry_msgs::Twist twist_msg);
        ~Action() override;
        void operate() override;
        void stop() override;
        bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) override;
    private:
        ros::Publisher& publisher;
        geometry_msgs::Twist twist_msg;
    };
} // namespace base_operator