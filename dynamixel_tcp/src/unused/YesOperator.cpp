#include <ros/ros.h>
#include "dynamixel_operator/Operators.hpp"
#include <thread>
#include <chrono>

namespace dynamixel_tcp
{
    YesOperator::YesOperator(ros::Publisher &target_publisher) : target_publisher(target_publisher), head_status(UP)
    {
        target_state.name.push_back(id);
        target_state.position.push_back(0);
        target_state.velocity.push_back(0);
    }
    YesOperator::~YesOperator()
    {
    }
    void YesOperator::operate()
    {
        target_state.velocity[0] = MOVING_SPEED;
        cancel_triggered = false;
        while (!cancel_trigger)
        {
            switch (head_status)
            {
                case UP:
                    target_state.position[0] = DOWN_VALUE;
                    target_publisher.publish(target_state);
                    head_status = DOWN;
                    break;
                case DOWN:
                    target_state.position[0] = UP_VALUE;
                    target_publisher.publish(target_state);
                    head_status = UP;
                    break;
                default:
                    ROS_ERROR("Unknown State");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_TIME));
        }
        /*Set to Original Velocity*/
        target_state.velocity[0] = MOVING_SPEED;
        target_publisher.publish(target_state);
    }
    void YesOperator::cancel(){
        cancel_triggered = true
    }
}