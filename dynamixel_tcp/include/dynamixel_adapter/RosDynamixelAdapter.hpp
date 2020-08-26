#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "dynamixel_adapter/dynamixel_adapter.hpp"

namespace dynamixel_tcp
{
    class RosDynamixelAdapter
    {
    public:
        RosDynamixelAdapter(ros::NodeHandle &nodeHandle);
        virtual ~RosDynamixelAdapter();

    private:
        bool readParameters();
        void latchPositionCallback(const sensor_msgs::JointState &joint_state);
        /*void setVelocityCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);*/
        //Velocity setting is constant at initialization

        ros::NodeHandle &nh;
        ros::Subscriber target_subscriber;
        ros::Publisher current_publisher;

        sensor_msgs::JointState desired_position;
        sensor_msgs::JointState current_position;

        std::string ip_addr;
        int port;
        std::vector<int> id_list;
        std::vector<int> cw_lim_list;
        std::vector<int> ccw_lim_list;

        DynamixelAdapter dynamixel_adapter;
    };

} // namespace dynamixel_tcp