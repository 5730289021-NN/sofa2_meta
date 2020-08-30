#include <dynamixel_operator/Operators.hpp>
#include <ros/ros.h>

namespace dynamixel_tcp
{
    ToGoalOperator::ToGoalOperator(ros::Publisher &target_publisher, std::vector<std::string> &ids, std::vector<double> &goals, std::vector<double> &moving_speeds)
    : Operator(target_publisher)
    {
        for(int i = 0; i < ids.size(); i++)
        {
            target_state_msg.name.push_back(ids[i]);
            target_state_msg.position.push_back(goals[i]);
            target_state_msg.velocity.push_back(moving_speeds[i]);
        } 
    }
    ToGoalOperator::~ToGoalOperator()
    {
    }
    void ToGoalOperator::operate()
    {
        target_publisher.publish(target_state_msg);
    }

} // namespace dynamixel_tcp