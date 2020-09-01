#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include "base_operator/Action.hpp"

namespace base_operator
{
    Action::Action(ros::Publisher &publisher, geometry_msgs::Twist twist_msg): publisher(publisher), twist_msg(twist_msg)
    {
    }
    Action::~Action()
    {
    }
    void Action::operate()
    {
        publisher.publish(twist_msg);
    }
    void Action::stop()
    {
        geometry_msgs::Twist stop_msg;
        publisher.publish(stop_msg);
    }

    bool Action::serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        operate();
    }
} // namespace base_operator