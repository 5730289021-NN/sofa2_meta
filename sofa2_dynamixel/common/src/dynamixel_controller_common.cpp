/**
* @file dynamixel_controller_common.cpp
* @author Norawit Nangsue
*
* Copyright (C) FIBO
*
* @brief A Dynamixel Controller for SOFA-2 Robot
*        This file is to be edited by the Developer
**/

// ROS message includes
#include "ros/ros.h"

// ROS message & services includes
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>


/* protected region user include files begin */
//Regenerate project requires correcting the CMakeLists.txt
#include <async_comm/tcp_client.h>
#include <iostream>
/* protected region user include files end */

/**
 * @class DynamixelControllerConfig
 * @brief set of static and dynamic parameters
 * @warning this class is autogenerated. It should not be touched!
 */
class DynamixelControllerConfig
{
public:
    // parameters handled through the parameter server
    int hpai;
    int hyai;
    int cpuai;
    int cpdai;
    //! overloading the print operator
    friend std::ostream& operator<< (std::ostream& os,
                                     const DynamixelControllerConfig& config)
    {
        os << "hpai: " << config.hpai << std::endl;
        os << "hyai: " << config.hyai << std::endl;
        os << "cpuai: " << config.cpuai << std::endl;
        os << "cpdai: " << config.cpdai << std::endl;
        return os;
    }
};

/**
 * @class DynamixelControllerData
 * @brief set of input / output handled through the update methods
 * @warning this class is autogenerated. It should not be touched!
 */
class DynamixelControllerData
{
public:
    // input data
    sensor_msgs::Joy in_joy;
    bool in_joy_updated;
    // output data
    sensor_msgs::JointState out_joint_state;
    bool out_joint_state_active;
};

/**
 * @class DynamixelControllerImpl
 * @brief Implementation of the node intelligence
 * @warning this class is be filled by the Developer, at locations indicated
 */
class DynamixelControllerImpl
{
    /* protected region user member variables begin */
    /* protected region user member variables end */

public:
    /**
     * @brief constructor
     */
    DynamixelControllerImpl()
    {
        /* protected region user constructor begin */
        ROS_INFO("Hey0");
        /* protected region user constructor end */
    }
    /**
     * @brief destructor
     */
    ~DynamixelControllerImpl()
    {
        /* protected region user destructor begin */
        /* protected region user destructor end */
    }
    /**
     * @brief method called at node configuration
     * @param config set of configuration parameters available
     */
    void configure(DynamixelControllerConfig config)
    {
        /* protected region user configure begin */
        ROS_INFO("Hey");
        /* protected region user configure end */
    }
    /**
     * @brief Update method periodically called by the ros component
     * @param data contains received messages (through subscription), and will contain messages to publish
     * @param config latest state of the config variables
     */
    void update(DynamixelControllerData &data, DynamixelControllerConfig config)
    {
        /* protected region user update begin */
        /* protected region user update end */
    }

    /* protected region user additional functions begin */
    /* protected region user additional functions end */
};