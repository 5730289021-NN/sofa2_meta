#include <dynamixel_operator/Operators.hpp>

namespace dynamixel_tcp
{
    SingleJointOperator::SingleJointOperator(ros::Publisher &target_publisher, int id, int cw_lim_value, int ccw_lim_value, int sleep_time_millis, int moving_speed)
        : target_publisher(target_publisher), id(id), cw_lim_value(cw_lim_value), ccw_lim_value(ccw_lim_value), sleep_time_millis(sleep_time_millis), moving_speed(moving_speed), joint_status(X)
    {
        target_state_msg.name.push_back(id);
        target_state_msg.position.push_back((cw_lim_value + ccw_lim_value) / 2);
        target_state_msg.velocity.push_back(moving_speed);
    }

    SingleJointOperator::~SingleJointOperator()
    {
    }

    void SingleJointOperator::operate()
    {
        cancel_triggered = false;
        while (!cancel_trigger)
        {
            switch (joint_status)
            {
            case X:
                target_state_msg.position[0] = Y;
                target_publisher.publish(target_state_msg);
                head_status = Y;
                break;
            case Y:
                target_state_msg.position[0] = X;
                target_publisher.publish(target_state_msg);
                head_status = X;
                break;
            default:
                ROS_ERROR("Unknown State");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
        }
    }

    void SingleJointOperator::cancel() {
        cancel_triggered = true;
    }
} // namespace dynamixel_tcp