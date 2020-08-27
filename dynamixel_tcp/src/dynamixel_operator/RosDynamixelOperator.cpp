#include <dynamixel_operator/RosDynamixelOperator.hpp>
#include <ros/ros.h>
#include <vector>

namespace dynamixel_tcp
{
    RosDynamixelOperator::RosDynamixelOperator(ros::NodeHandle &nodeHandle)
        : nh(nodeHandle), head_shaking(false)
    {
        if (!readParameters())
        {
            ROS_ERROR("ROS Initialization Failed");
            return;
        }

        shakehead_srv = nh.advertiseService("/head/operator/shakehead",
                                            &RosDynamixelOperator::shakeHeadServiceCallback, this);

        target_publisher = nh.advertise<sensor_msgs::JointState>("/head/target_position", 3);
        current_subscriber = nh.subscribe("/head/current_position", 3,
                                          &RosDynamixelAdapter::latchPositionCallback, this);

        ros::Rate loop_rate(10);
        /*Finite State Machine Here*/
        while (ros::ok())
        {
            if (head_shaking && isTargetPositionArrived())
            {
                changeTargetShakeHeadPosition();
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    bool RosDynamixelOperator::isTargetPositionArrived() const
    {
        /*The yaw of head must be at index 1*/
        return (current_state.position[1] < desired_state.position[1] + TOLERANCE &&
                current_state.position[1] > desired_state.position[1] - TOLERANCE)
    }

    bool RosDynamixelOperator::shakeHeadServiceCallback(std_srvs::Trigger::Request &request,
                                                        std_srvs::Trigger::Response &response)
    {
        if (request.data)
        {
            head_shaking = true;
            response.success = true;
            response.message = "The head is shaking";
        }
        else
        {
            head_shaking = false;
            moveToHome();
            response.success = true;
            response.message = "The head is going back to home";
        }
        return true;
    }

    void RosDynamixelOperator::changeTargetShakeHeadPosition()
    {
        double goal_position;
        if (shake_head_state == LEFT)
        {
            shake_head_state = RIGHT;
            goal_position = shake_head_ccw_lim;
        }
        else
        {
            shake_head_state = LEFT;
            goal_position = shake_head_cw_lim;
        }
        std::vector<std::string> names{"1"};
        std::vector<double> positions{goal_position};
        target_state.name = names;
        target_state.position = positions;
        target_publisher.publish(target_state);
    }

    void RosDynamixelOperator::moveToHome()
    {
        target_publisher.publish(home_state);
    }
} // namespace dynamixel_tcp