#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "dynamixel_adapter/dynamixel_adapter.hpp"

#include "dynamixel_adapter/RosDynamixelAdapter.hpp"


namespace dynamixel_tcp
{
    RosDynamixelAdapter::RosDynamixelAdapter(ros::NodeHandle &nodeHandle)
    : nh(nodeHandle)
    {
        if (!readParameters())
        {
            ROS_ERROR("ROS Initialization Failed");
            return;
        }

        target_subscriber = nh.subscribe("/head/target_position", 3,
                                                  &RosDynamixelAdapter::latchPositionCallback, this);

        current_publisher = nh.advertise<sensor_msgs::JointState>("/head/current_position", 3);

        ros::Rate loop_rate(10); // Should be enough

        /*Config Kp, Ki, Kd then Enable Torque*/
        dynamixel_adapter = DynamixelAdapter(ip_addr, port, id_list, cw_lim_list, ccw_lim_list, kp_list, ki_list, kd_list); //RAI (connect + config dynamixel)

        if (!dynamixel_adapter.getErrorMsg().empty())
        {
            /*TODO: Set Fault Flag*/
            ROS_ERROR_STREAM(dynamixel_adapter.getErrorMsg());
        }

        while (ros::ok())
        {
            /*Read Current Position*/
            current_position.position = dynamixel_adapter.readPositions();
            /*Write Desire Position which is latched from latchPositionCallback*/
            dynamixel_adapter.writePositions(desired_position.name, desired_position.position);
            /*Publish*/
            current_publisher.publish(current_position);
            /*Checking for message*/
            ros::spinOnce();
            /*Delay*/
            loop_rate.sleep();
        }
    }

    RosDynamixelAdapter::~RosDynamixelAdapter()
    {
    }

    void RosDynamixelAdapter::latchPositionCallback(const sensor_msgs::JointState &joint_state)
    {
        /*Latch Desire Position*/
        desired_position = joint_state;
    }

    bool RosDynamixelAdapter::readParameters()
    {
        nh.param<std::string>("dynamixel_controller/ip_addr", ip_addr, "192.168.16.23");
        nh.param("dynamixel_controller/port", port, 5002);
        // nh.param("dynamixel_controller/spin_rate", spin_rate, 5);
        // nh.param("dynamixel_controller/enb_index", enb_index, 6);

        bool success = true;

        if (!nh.getParam("dynamixel_controller/id", id_list))
        {
            ROS_ERROR("No Dynamixel ID Given!");
            success = false;
        }
        if (!nh.getParam("dynamixel_controller/cw_lim", cw_lim_list))
        {
            ROS_ERROR("No Dynamixel Clockwise Limit Given!");
            success = false;
        }
        if (!nh.getParam("dynamixel_controller/ccw_lim", ccw_lim_list))
        {
            ROS_ERROR("No Dynamixel Counter-Clockwise Limit Given!");
            success = false;
        }
        if (!nh.getParam("dynamixel_controller/kp", kp_list))
        {
            ROS_ERROR("No Dynamixel Kp Given!");
            success = false;
        }
        if (!nh.getParam("dynamixel_controller/ki", ki_list))
        {
            ROS_ERROR("No Dynamixel Ki Given!");
            success = false;
        }
        if (!nh.getParam("dynamixel_controller/kd", kd_list))
        {
            ROS_ERROR("No Dynamixel Kd Given!");
            success = false;
        }

        int m = id_list.size();
        if (m != cw_lim_list.size() || m != ccw_lim_list.size())
        {
            ROS_ERROR("Dynamixel Parameter Size Mismatched");
            success = false;
        }

        return success;
    }

} // namespace dynamixel_tcp