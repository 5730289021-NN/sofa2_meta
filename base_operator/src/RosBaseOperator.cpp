#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include "base_operator/RosBaseOperator.hpp"
#include "base_operator/Operation.hpp"
#include "base_operator/Action.hpp"
#include "base_operator/Sequence.hpp"
#include <utility>
#include <vector>
#include <string>
#include <ros/ros.h>

namespace base_operator
{
    RosBaseOperator::RosBaseOperator(ros::NodeHandle &nodeHandle)
        : nh(nodeHandle)
    {
        publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        if (!initializeServices(publisher))
        {
            ROS_ERROR("Base Operator Service Initialization Failed");
            return;
        }
        ROS_INFO("Base Operator Started");

        ros::spin();
        ROS_ERROR("Base Operator Terminated");
    }

    RosBaseOperator::~RosBaseOperator()
    {
    }

    bool RosBaseOperator::initializeServices(ros::Publisher &publisher)
    {
        bool success = true;

        if (!nh.getParam("/base_operator/operator_names", operator_names))
        {
            ROS_ERROR("Base Operator Name Given!");
            success = false;
        }

        if (!nh.getParam("/base_operator/operator_types", operator_types))
        {
            ROS_ERROR("Base Operator Type Given!");
            success = false;
        }
        int operator_size = operator_names.size();

        if (operator_size != operator_types.size())
        {
            ROS_ERROR("Base Operator Size Mismatched");
        }

        for (int i = 0; i < operator_size; i++)
        {
            if (operator_types[i].compare("action") == 0)
            {
                double vx, wz;
                if (!nh.getParam("/base_operator/" + operator_names[i] + "/vx", vx))
                    ;
                {
                    ROS_ERROR("Base Operator vx doen't given");
                    success = false;
                }
                if (!nh.getParam("/base_operator/" + operator_names[i] + "/wz", wz))
                    ;
                {
                    ROS_ERROR("Base Operator wz doen't given");
                    success = false;
                }
                if (success)
                {
                    geometry_msgs::Twist twist_msg;
                    twist_msg.linear.x = vx;
                    twist_msg.angular.z = wz;
                    Operation *action_op = new Action(publisher, twist_msg);
                    operator_map[operator_names[i]] = action_op;
                    services.push_back(nh.advertiseService("/base/operator/" + operator_names[i], &Operation::serviceCallback, action_op));
                    ROS_INFO_STREAM("/base/operator/" << operator_names[i] << " service initiated");
                }
            }
            else if (operator_types[i].compare("sequence") == 0)
            {
                std::vector<std::string> names_;
                if (!nh.getParam("/base_operator/" + operator_names[i] + "/sequence", names_))
                {
                    ROS_ERROR("Base Operator sequence doen't given");
                    success = false;
                }

                std::vector<double> times_;
                if (!nh.getParam("/base_operator/" + operator_names[i] + "/time", times_))
                {
                    ROS_ERROR("Base Operator time doen't given");
                    success = false;
                }

                int rep_;
                if (!nh.getParam("/base_operator/" + operator_names[i] + "/repetition", rep_))
                {
                    ROS_ERROR("Base Operator repetition doen't given");
                    success = false;
                }
                if (success)
                {
                    std::vector<std::pair<Action *, double>> sequence_time;
                    for (int i = 0; i < names_.size(); i++)
                    {
                        std::pair<Action *, double> p;
                        p.first = static_cast<Action*>(operator_map[names_[i]]);
                        p.second = times_[i];
                        sequence_time.push_back(p);
                    }

                    Operation *sequence_op = new Sequence(nh, sequence_time, rep_);
                    operator_map[operator_names[i]] = sequence_op;
                    services.push_back(nh.advertiseService("/base/operator/" + operator_names[i], &Operation::serviceCallback, sequence_op));
                    ROS_INFO_STREAM("/base/operator/" << operator_names[i] << " service initiated");
                }
            }
            else
            {
                ROS_ERROR_STREAM("Unknown Type Error " << operator_types[i]);
                success = false;
            }
        }
        return success;
    }
} // namespace base_operator