#pragma once
#include <std_srvs/SetBool.h>
#include "base_operator/Operation.hpp"
#include "base_operator/Action.hpp"
#include <ros/ros.h>
#include <vector>
#include <utility> 

namespace base_operator
{
    class Sequence : public Operation
    {
    public:
        Sequence(ros::NodeHandle &nodeHandle, std::vector<std::pair<Action *, double>> sequence, int repetition);
        ~Sequence() override;
        void operate() override;
        void stop() override;
        bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) override;

        void timerCallback(const ros::TimerEvent &time_event);
    
    private:
        ros::NodeHandle& nh;
        ros::Timer timer;
        int repetition;
        int counter;
        int sequence_index;
        std::vector<std::pair<Action*, double>> sequence;
    };
} // namespace base_operator