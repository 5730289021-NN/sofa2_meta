/**
* @file follow_me_ros.cpp
* @author Norawit Nangsue
*
* Copyright (C) FIBO
*
* @brief follow_me
* @warning This file should not be edited
**/

// ROS includes
#include <ros/ros.h>

// ROS message & services includes
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <object_msgs/ObjectsInBoxes.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/SetBool.h>

// other includes
#include <follow_me_common.cpp>

/**
 * @class FollowMeROS
 * @brief Class handling the connection with the ROS world.
 * It also implement the node life-cycle, and access to object follow_me-impl when appropriate
 */
class FollowMeROS
{
public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    ros::Publisher cmd_vel_;
    ros::Publisher status_;
    ros::Subscriber detected_people_;
    ros::Subscriber depth_image_;
    ros::ServiceServer command_;

    FollowMeData component_data_;
    // todo confirm it should be always defined, even if not used.
    FollowMeConfig component_config_;
    FollowMeImpl component_implementation_;

    /**
     * @brief object constructor.
     */
    FollowMeROS() : np_("~")
    {
        cmd_vel_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        status_ = n_.advertise<std_msgs::String>("status", 1);
        detected_people_ = n_.subscribe("detected_people", 1, &FollowMeROS::topicCallback_detected_people, this);
        depth_image_ = n_.subscribe("depth_image", 1, &FollowMeROS::topicCallback_depth_image, this);
        // handling parameters from the parameter server
        np_.param("Kp_x", component_config_.Kp_x, (double)1);
        np_.param("Kp_a", component_config_.Kp_a, (double)1);
        np_.param("vx_max", component_config_.vx_max, (double)0.5);
        np_.param("wz_max", component_config_.wz_max, (double)0.6);
        np_.param("min_dist_fol", component_config_.min_dist_fol, (double)1.5);
        np_.param("max_dist_fol", component_config_.max_dist_fol, (double)4.0);
        np_.param("prob_thres", component_config_.prob_thres, (double)0.9);
        // handling Service servers
        command_ = n_.advertiseService<std_srvs::SetBool::Request , std_srvs::SetBool::Response>("command", boost::bind(&FollowMeImpl::callback_command, &component_implementation_, _1, _2, &component_config_));
    }

    /**
     * @brief callback of a topic subscription handled through the update mechanism.
     * @param msg message received from ROS world
     */
    void topicCallback_detected_people(const object_msgs::ObjectsInBoxes::ConstPtr& msg)
    {
        component_data_.in_detected_people = *msg;
        component_data_.in_detected_people_updated = true;
    }

    /**
     * @brief callback of a topic subscription handled through the update mechanism.
     * @param msg message received from ROS world
     */
    void topicCallback_depth_image(const sensor_msgs::Image::ConstPtr& msg)
    {
        component_data_.in_depth_image = *msg;
        component_data_.in_depth_image_updated = true;
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
        component_data_.out_cmd_vel_active = true;
        component_data_.out_status_active = true;
    }
    /**
     * @brief State that all input has been read
     */
    void all_input_read()
    {
        component_data_.in_detected_people_updated = false;
        component_data_.in_depth_image_updated = false;
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
        if (component_data_.out_cmd_vel_active)
            cmd_vel_.publish(component_data_.out_cmd_vel);
        if (component_data_.out_status_active)
            status_.publish(component_data_.out_status);
    }
    /**
     * @brief object destructor
     */
    ~FollowMeROS()
    {
    }
};

/**
 * @brief Main of the component. Create the object and launches the periodical call to upde method.
 */
int main(int argc, char** argv)
{

    ros::init(argc, argv, "follow_me");

    ros::AsyncSpinner spinner(1);

    FollowMeROS node;
    node.configure();

    ros::Timer timer = node.n_.createTimer(ros::Duration(1.0 / 20), &FollowMeROS::update, &node);

    spinner.start();

    ros::waitForShutdown();

    return 0;
}
