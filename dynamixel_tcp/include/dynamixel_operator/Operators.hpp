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
        Operator() : isRunning(false) {}
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
                    std::thread caller(operate);
                    res.success = true;
                    res.message = "Successfully start thread";
                }
                else{
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

    private:
        ros::Publisher &target_publisher
            sensor_msgs::JointState target_state_msg;
        std::thread caller;
        bool isRunning;
    };

    class SingleJointOperator : public Operator
    {
    public:
        SingleJointOperator(ros::Publisher &target_publisher, int id, int cw_lim_value, int ccw_lim_value, int sleep_time_millis, int moving_speed);
        ~SingleJointOperator();
        void operate();
        void cancel();

    private:
        int id;
        int cw_lim_value;
        int ccw_lim_value;
        int sleep_time_millis;
        int moving_speed;
        volatile bool cancel_triggered;
        enum JointStatus
        {
            X,
            Y
        } JointStatus joint_status;
    };

    class ToGoalOperator : public Operator
    {
    public:
        ToGoalOperator(ros::Publisher &target_publisher, std::vector<std::string> &ids, std::vector<double> &goals, std::vector<double> &moving_speeds);
        ~ToGoalOperator();
        void operate();
    };
} // namespace dynamixel_tcp