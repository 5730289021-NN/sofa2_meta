#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
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
        ROS_INFO_STREAM("Initializing Dynamixel Adapter");
        dynamixel_adapter = new DynamixelAdapter(ip_addr, port, id_list, cw_lim_list, ccw_lim_list, kp_list, ki_list, kd_list); //RAI (connect + config dynamixel)
        std::vector<std::string> id_names;
        for (auto id : id_list)
        {
            id_names.push_back(std::to_string(id));
        }
        current_position.name = id_names;

        ROS_INFO_STREAM("Checking Dynamixel Error Message...");
        if (!dynamixel_adapter->getErrorMsg().empty())
        {
            /*TODO: Set Fault Flag*/
            ROS_ERROR_STREAM(dynamixel_adapter->getErrorMsg());
        }

        ROS_INFO_STREAM("Dynamixel TCP: Starting the loop");

        while (ros::ok())
        {
            if(dynamixel_adapter->getStatus()){
                /*Read Current Position*/
                current_position.position = dynamixel_adapter->readPositions();
                /*Write Desire Position which is latched from latchPositionCallback*/
                if (target_updated)
                {
                    //ROS_INFO_STREAM("Getting Updated: " << desired_position.name[0] << " " << desired_position.position[0]);
                    int id_size = desired_position.name.size();
                    if (id_size == desired_position.position.size() && id_size == desired_position.velocity.size())
                    {
                        dynamixel_adapter->writePositionsVelocities(desired_position.name, desired_position.position, desired_position.velocity);
                    }
                    else if (id_size == desired_position.position.size())
                    {
                        dynamixel_adapter->writePositions(desired_position.name, desired_position.position);
                    }
                    else
                    {
                        ROS_ERROR("Unable to perform input task");
                    }
                    target_updated = false;
                }
            } else {
                dynamixel_adapter->tryReconnect();
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }


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
        delete dynamixel_adapter;
    }

    void RosDynamixelAdapter::latchPositionCallback(const sensor_msgs::JointState &joint_state)
    {
        /*Latch Desire Position*/
        target_updated = true;
        desired_position = joint_state;
    }

    bool RosDynamixelAdapter::readParameters()
    {
        nh.param<std::string>("/dynamixel_adapter/ip_addr", ip_addr, "192.168.16.23");
        nh.param("/dynamixel_adapter/port", port, 9002);
        // nh.param("dynamixel_controller/spin_rate", spin_rate, 5);
        // nh.param("dynamixel_controller/enb_index", enb_index, 6);

        bool success = true;

        if (!nh.getParam("/dynamixel_adapter/id", id_list))
        {
            ROS_ERROR("No Dynamixel ID Given!");
            success = false;
        }
        if (!nh.getParam("/dynamixel_adapter/cw_lim", cw_lim_list))
        {
            ROS_ERROR("No Dynamixel Clockwise Limit Given!");
            success = false;
        }
        if (!nh.getParam("/dynamixel_adapter/ccw_lim", ccw_lim_list))
        {
            ROS_ERROR("No Dynamixel Counter-Clockwise Limit Given!");
            success = false;
        }
        if (!nh.getParam("/dynamixel_adapter/kp", kp_list))
        {
            ROS_ERROR("No Dynamixel Kp Given!");
            success = false;
        }
        if (!nh.getParam("/dynamixel_adapter/ki", ki_list))
        {
            ROS_ERROR("No Dynamixel Ki Given!");
            success = false;
        }
        if (!nh.getParam("/dynamixel_adapter/kd", kd_list))
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