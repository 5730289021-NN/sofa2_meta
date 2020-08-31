#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <vector>
#include <string>
#include <thread>


namespace dynamixel_tcp
{
    class Operator
    {
    public:
        Operator(ros::Publisher &target_publisher);
        virtual ~Operator();
        virtual void operate() = 0;
        virtual bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    protected:
        ros::Publisher &target_publisher;
        sensor_msgs::JointState target_state_msg;

    private:
        std::thread caller;
        bool isRunning;
    };

    class SingleJointOperator : public Operator
    {
    public:
        SingleJointOperator(ros::NodeHandle &nh, ros::Publisher &target_publisher, std::string id, int cw_lim_value, int ccw_lim_value, int sleep_time_millis, int moving_speed);
        ~SingleJointOperator() override;
        void operate() override;
        bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) override;
        void timerCallback(const ros::TimerEvent&);

    private:
        std::string id;
        ros::NodeHandle &nh;
        int cw_lim_value;
        int ccw_lim_value;
        int sleep_time_millis;
        int moving_speed;
        enum JointStatus
        {
            X,
            Y
        };

        ros::Timer timer;
        JointStatus joint_status;
    };

    class ToGoalOperator : public Operator
    {
    public:
        ToGoalOperator(ros::Publisher &target_publisher, std::vector<std::string> &ids, std::vector<double> &goals, std::vector<double> &moving_speeds, int timeout);
        ~ToGoalOperator() override;
        bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) override;
        void operate() override;
        int getTimeout() const;
        sensor_msgs::JointState& getMessage();
    private:
        int timeout;
    };

    class SequenceOperator : public Operator
    {
    public:
        SequenceOperator(ros::NodeHandle &nh, ros::Publisher &target_publisher, std::vector<ToGoalOperator*> to_goal_operators);
        ~SequenceOperator() override;
        void operate() override;
        bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) override;
        void timerCallback(const ros::TimerEvent&);
    private:
        ros::NodeHandle &nh;
        ros::Timer timer;
        std::vector<ToGoalOperator*> to_goal_operators;
        int current_operator_index = 0;
    };
} // namespace dynamixel_tcp