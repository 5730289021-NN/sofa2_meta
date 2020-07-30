/**
* @file dynamixel_controller_ros.cpp
* @author Norawit Nangsue
*
* Copyright (C) FIBO
*
* @brief A Dynamixel Controller for SOFA-2 Robot
* @warning This file should not be edited
**/

// ROS includes
#include <ros/ros.h>

// ROS message & services includes
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

// other includes
#include <dynamixel_controller_common.cpp>

/**
 * @class DynamixelControllerROS
 * @brief Class handling the connection with the ROS world.
 * It also implement the node life-cycle, and access to object dynamixel_controller-impl when appropriate
 */
class DynamixelControllerROS
{
public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    ros::Publisher joint_state_;
    ros::Subscriber joy_;

    DynamixelControllerData component_data_;
    // todo confirm it should be always defined, even if not used.
    DynamixelControllerConfig component_config_;
    DynamixelControllerImpl component_implementation_;

    /**
     * @brief object constructor.
     */
    DynamixelControllerROS() : np_("~")
    {
        joint_state_ = n_.advertise<sensor_msgs::JointState>("joint_state", 1);
        joy_ = n_.subscribe("joy", 1, &DynamixelControllerROS::topicCallback_joy, this);
        // handling parameters from the parameter server
        np_.param("hpai", component_config_.hpai, (int)4);
        np_.param("hyai", component_config_.hyai, (int)3);
        np_.param("cpuai", component_config_.cpuai, (int)2);
        np_.param("cpdai", component_config_.cpdai, (int)5);
    }

    /**
     * @brief callback of a topic subscription handled through the update mechanism.
     * @param msg message received from ROS world
     */
    void topicCallback_joy(const sensor_msgs::Joy::ConstPtr& msg)
    {
        component_data_.in_joy = *msg;
        component_data_.in_joy_updated = true;
    }

    /**
     * @brief configure function called after node creation.
     */
    void configure()
    {
        component_implementation_.configure(component_config_);
    }
    /**
     * @brief Activate all publishers handled through the update mechanism
     */
    void activate_all_output()
    {
        component_data_.out_joint_state_active = true;
    }
    /**
     * @brief State that all input has been read
     */
    void all_input_read()
    {
        component_data_.in_joy_updated = false;
     }
    /**
     * @brief core function periodically called.
     * calls implementation update, and handles potential publications
     * @param event access to the timer used for the looping
     */
    void update(const ros::TimerEvent& event)
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        all_input_read();
        if (component_data_.out_joint_state_active)
            joint_state_.publish(component_data_.out_joint_state);
    }
    /**
     * @brief object destructor
     */
    ~DynamixelControllerROS()
    {
    }
};

/**
 * @brief Main of the component. Create the object and launches the periodical call to upde method.
 */
int main(int argc, char** argv)
{

    ros::init(argc, argv, "dynamixel_controller");

    ros::AsyncSpinner spinner(1);

    DynamixelControllerROS node;
    node.configure();

    ros::Timer timer = node.n_.createTimer(ros::Duration(1.0 / 20), &DynamixelControllerROS::update, &node);

    spinner.start();

    ros::waitForShutdown();

    return 0;
}
