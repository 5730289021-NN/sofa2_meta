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
        Operator(ros::Publisher &target_publisher) : isRunning(false), target_publisher(target_publisher) {}
        virtual ~Operator() {}
        virtual void operate() = 0;
        virtual void cancel(){};
        bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
        {
            if (req.data)
            {
                if (!isRunning)
                {
                    isRunning = true;
                    std::thread caller(&Operator::operate, this);
                    res.success = true;
                    res.message = "Successfully start thread";
                }
                else
                {
                    res.success = false;
                    res.message = "This operator is still running";
                }
            }
            else
            {
                isRunning = false;
                cancel();
                res.success = true;
                res.message = "Successfully stop thread";
            }
        }

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
        SingleJointOperator(ros::Publisher &target_publisher, std::string id, int cw_lim_value, int ccw_lim_value, int sleep_time_millis, int moving_speed);
        ~SingleJointOperator() override;
        void operate() override;
        void cancel() override;

    private:
        std::string id;
        int cw_lim_value;
        int ccw_lim_value;
        int sleep_time_millis;
        int moving_speed;
        volatile bool cancel_triggered;
        enum JointStatus
        {
            X,
            Y
        };
        JointStatus joint_status;
    };

    class ToGoalOperator : public Operator
    {
    public:
        ToGoalOperator(ros::Publisher &target_publisher, std::vector<std::string> &ids, std::vector<double> &goals, std::vector<double> &moving_speeds);
        ~ToGoalOperator() override;
        void operate() override;
    };
} // namespace dynamixel_tcp