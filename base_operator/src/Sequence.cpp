#include <std_srvs/SetBool.h>
#include "base_operator/Sequence.hpp"
#include "base_operator/Action.hpp"
#include <ros/ros.h>
#include <vector>
#include <utility>

namespace base_operator
{
    Sequence::Sequence(ros::NodeHandle &nodeHandle,std::vector<std::pair<Action *, double>> sequence, int repetition): nh(nodeHandle), repetition(repetition), counter(0)
    {
        timer = nh.createTimer(ros::Duration(1.0), &Sequence::timerCallback, this, false, false);
        this->sequence = std::move(sequence);
    }
    Sequence::~Sequence()
    {
        for (auto p : sequence)
        {
            delete p.first;
        }
    }
    void Sequence::operate()
    {
        counter = 0;
        sequence_index = 0;
        sequence[sequence_index].first->operate();
        timer.setPeriod(ros::Duration(sequence[0].second));
        timer.start();
    }
    bool Sequence::serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        //Activate Timer
        if (req.data)
        {
            if (timer.hasStarted())
            {
                res.success = false;
                res.message = "Unable to start, Sequence still in progressed";
            }
            else
            {
                res.success = true;
                res.message = "Sequence started";
                operate(); //run immediately
            }
        }
        else
        {
            if (timer.hasStarted())
            {
                res.success = true;
                res.message = "Successfully stop Sequence";
                stop();
            }
            else
            {
                res.success = false;
                res.message = "Sequence already stopped";
            }
        }
        return true;
    }

    void Sequence::timerCallback(const ros::TimerEvent &time_event)
    {
        sequence_index++;
        if (sequence_index == sequence.size())
        {
            sequence_index = 0;
            counter++;
            if (counter >= repetition)
            {
                stop();
            }
        }
        timer.setPeriod(ros::Duration(sequence[sequence_index].second));
        sequence[sequence_index].first->operate();
    }

    void Sequence::stop(){
        timer.stop();
        //Could be any index
        sequence[0].first->stop();
    }
} // namespace base_operator