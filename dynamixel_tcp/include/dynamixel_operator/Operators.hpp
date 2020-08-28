#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>

namespace dynamixel_tcp
{
    class Operator
    {
    public:
        Operator() {}
        virtual ~Operator() {}
        virtual void operate() = 0;

    private:
        ros::Publisher &target_publisher;
        sensor_msgs::JointState target_state_msg;
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

    /*Every Yes and No Operator Require HeadHome to be done first*/
    // class NoOperator : public Operator
    // {
    // public:
    //     NoOperator(ros::Publisher& target_publisher);
    //     ~NoOperator();
    //     void operate();
    //     void cancel();
    // private:
    //     static const std::string id = '2';
    //     static const int LEFT_VALUE = 2700;
    //     static const int RIGHT_VALUE = 1400;
    //     static const int SLEEP_TIME_MILLIS = 1000
    //     static const int MOVING_SPEED = 10;
    //     enum HeadStatus{LEFT, RIGHT}
    //     HeadStatus head_status;
    // };

    // class YesOperator : public Operator
    // {
    // public:
    //     YesOperator(ros::Publisher& target_publisher);
    //     ~YesOperator();
    //     void operate();
    //     void cancel();
    // private:
    //     static const std::string id = '1';
    //     static const int UP_VALUE = 1950;
    //     static const int DOWN_VALUE = 2020;
    //     static const int SLEEP_TIME_MILLIS = 1000
    //     static const int MOVING_SPEED = 10;
    //     enum HeadStatus{UP, DOWN}
    //     HeadStatus head_status;
    // };

    // class HeadHomeOperator : public Operator
    // {
    // public:
    //     HeadHomeOperator(ros::Publisher& target_publisher);
    //     ~HeadHomeOperator();
    //     void operate();
    // };

    // class CameraHomeOperator : public Operator
    // {
    // public:
    //     CameraHomeOperator(ros::Publisher& target_publisher);
    //     ~CameraHomeOperator();
    //     void operate();
    // private:
    //     static const int std::string id = '0';
    // };
} // namespace dynamixel_tcp