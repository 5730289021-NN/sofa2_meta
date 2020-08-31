#include <dynamixel_operator/Operators.hpp>
#include <ros/ros.h>

namespace dynamixel_tcp
{
    ToGoalOperator::ToGoalOperator(ros::Publisher &target_publisher, std::vector<std::string> &ids, std::vector<double> &goals, std::vector<double> &moving_speeds, int timeout)
    : Operator(target_publisher), timeout(timeout)
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

    bool ToGoalOperator::serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if(req.data){
            operate();
            res.success = true;
            res.message = "To Goal Operator Started";
        }
        else {
            res.success = false;
            res.message = "To Goal Operator not Started";
        }
        return true;
    }

    sensor_msgs::JointState& ToGoalOperator::getMessage() {
        return target_state_msg;
    }

    void ToGoalOperator::operate()
    {
        ROS_INFO("Call To Goal Operator");
        target_publisher.publish(target_state_msg);
    }

    int ToGoalOperator::getTimeout() const
    {
        return timeout;
    }

} // namespace dynamixel_tcp