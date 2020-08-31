#include "dynamixel_operator/Operators.hpp"
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <thread>

namespace dynamixel_tcp
{
    Operator::Operator(ros::Publisher &target_publisher) : isRunning(false), target_publisher(target_publisher) {}
    Operator::~Operator() {}
    bool Operator::serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {/*Actually not used because of runtime error*/
        if (req.data)
        {
            ROS_INFO_STREAM("Service Called with req.data: true");
            if (!isRunning)
            {
                ROS_INFO_STREAM("and not running");
                isRunning = true;
                std::thread caller(&Operator::operate, this);
                res.success = true;
                res.message = "Successfully start thread";
                return true;
            }
            else
            {
                res.success = false;
                res.message = "This operator is still running";
            }
        }
        else
        {
            ROS_INFO_STREAM("Service Called with req.data: false");
            isRunning = false;
            //cancel();
            res.success = true;
            res.message = "Successfully stop thread";
        }
    }
} // namespace dynamixel_tcp
