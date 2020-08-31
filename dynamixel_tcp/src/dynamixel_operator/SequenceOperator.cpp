#include <dynamixel_operator/Operators.hpp>
#include <ros/ros.h>

namespace dynamixel_tcp
{

/*  SequenceOperator(ros::NodeHandle &nh, ros::Publisher &target_publisher, std::vector<ToGoalOperator> to_goal_operators);
        ~SequenceOperator() override;
        void operate() override;
        bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) override;
        void timerCallback(const ros::TimerEvent&);*/

    SequenceOperator::SequenceOperator(ros::NodeHandle &nh, ros::Publisher &target_publisher, std::vector<ToGoalOperator*> to_goal_operators)
    : Operator(target_publisher), nh(nh), to_goal_operators(to_goal_operators), current_operator_index(0);
    {
        timer = nh.createTimer(ros::Duration(sleep_time_millis / 1000.0), &SingleJointOperator::timerCallback, this, false, false);
    }

    SequenceOperator::~SequenceOperator()
    {
        for(auto goal_op: to_goal_operators){
            delete goal_op;
        }
    }

    bool SequenceOperator::serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        //Activate Timer
        if (req.data)
        {
            if (timer.hasStarted())
            {
                res.success = false;
                res.message = "Unable to start, Operator still in progressed";
            }
            else
            {
                res.success = true;
                res.message = "Sequence Operator started";
                current_operator_index = 0;
                operate(); //run immediately
                timer.setPeriod(to_goal_operators[0]->getTimeout())
                timer.start();
            }
        }
        else
        {
            if (timer.hasStarted())
            {
                res.success = true;
                res.message = "Successfully stop Sequence Operator";
                timer.stop();
            }
            else
            {
                res.success = false;
                res.message = "Sequence Operator already stopped";
            }
        }
        return true;
    }

    void SequenceOperator::timerCallback(const ros::TimerEvent &time_event)
    {
        current_operator_index += 1;
        timer.setPeriod(to_goal_operators[current_operator_index]->getTimeout());
        operate();
    }

    void SequenceOperator::operate()
    {
        target_publisher.publish(to_goal_operators[current_operator_index]->getMessage());
    }

} // namespace dynamixel_tcp