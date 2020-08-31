#include <dynamixel_operator/Operators.hpp>

namespace dynamixel_tcp
{
    SingleJointOperator::SingleJointOperator(ros::NodeHandle &nodeHandle, ros::Publisher &target_publisher, std::string id, int cw_lim_value, int ccw_lim_value, int sleep_time_millis, int moving_speed)
        : Operator(target_publisher), nh(nodeHandle), id(id), cw_lim_value(cw_lim_value), ccw_lim_value(ccw_lim_value), sleep_time_millis(sleep_time_millis), moving_speed(moving_speed), joint_status(X)
    {
        target_state_msg.name.push_back(id);
        target_state_msg.position.push_back((cw_lim_value + ccw_lim_value) / 2);
        target_state_msg.velocity.push_back(moving_speed);
        timer = nh.createTimer(ros::Duration(sleep_time_millis / 1000.0), &SingleJointOperator::timerCallback, this, false, false);
    }

    SingleJointOperator::~SingleJointOperator()
    {
    }

    bool SingleJointOperator::serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
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
                res.message = "Operator started";
                operate(); //run immediately
                timer.start();
            }
        }
        else
        {
            if (timer.hasStarted())
            {
                res.success = true;
                res.message = "Successfully stop operator";
                timer.stop();
            }
            else
            {
                res.success = false;
                res.message = "Operator already stopped";
            }
        }
        return true;
    }

    void SingleJointOperator::timerCallback(const ros::TimerEvent &time_event)
    {
        operate();
    }

    void SingleJointOperator::operate()
    {
        switch (joint_status)
        {
        case X:
            target_state_msg.position[0] = cw_lim_value;
            ROS_INFO_STREAM("Sending Y: " << cw_lim_value);
            target_publisher.publish(target_state_msg);
            joint_status = Y;
            break;
        case Y:
            target_state_msg.position[0] = ccw_lim_value;
            ROS_INFO_STREAM("Sending X: " << ccw_lim_value);
            target_publisher.publish(target_state_msg);
            joint_status = X;
            break;
        default:
            ROS_ERROR("Unknown State");
        }
    }

} // namespace dynamixel_tcp
