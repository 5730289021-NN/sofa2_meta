#include <dynamixel_operator/RosDynamixelOperator.hpp>
#include <ros/ros.h>
#include <vector>
#include <string>

namespace dynamixel_tcp
{
    RosDynamixelOperator::RosDynamixelOperator(ros::NodeHandle &nodeHandle)
        : nh(nodeHandle)
    {
        target_state_publisher = nh.advertise<sensor_msgs::JointState>("/head/target_position", 1);
        if (!initializeServices(target_state_publisher))
        {
            ROS_ERROR("Dynamixel Operator Service Initialization Failed");
            return;
        }
        ROS_INFO("Dynamixel Operator Started");

        ros::Rate loop_rate(5);
        ros::spin();

        ROS_ERROR("Dynamixel Operator Terminated");
    }

    RosDynamixelOperator::~RosDynamixelOperator()
    {
    }

    bool RosDynamixelOperator::initializeServices(ros::Publisher& target_state_publisher)
    {
        bool success = true;

        if (!nh.getParam("/dynamixel_operator/operator_names", operator_names))
        {
            ROS_ERROR("No Dynamixel Operator Name Given!");
            success = false;
        }

        if (!nh.getParam("/dynamixel_operator/operator_types", operator_types))
        {
            ROS_ERROR("No Dynamixel Operator Type Given!");
            success = false;
        }
        int operator_size = operator_names.size();

        if (operator_size != operator_types.size())
        {
            ROS_ERROR("Dynamixel Operator Size Mismatched");
        }

        for (int i = 0; i < operator_size; i++)
        {
            if (operator_types[i].compare("goal") == 0)
            {
                std::vector<std::string> id_list;
                if (!nh.getParam("/dynamixel_operator/" + operator_names[i] + "/id", id_list))
                {
                    ROS_ERROR_STREAM("No id list for" << operator_names[i]);
                    success = false;
                }
                std::vector<double> goal_list;
                if (!nh.getParam("/dynamixel_operator/" + operator_names[i] + "/goal", goal_list))
                {
                    ROS_ERROR_STREAM("No goal list for" << operator_names[i]);
                    success = false;
                }
                std::vector<double> moving_speed;
                if (!nh.getParam("/dynamixel_operator/" + operator_names[i] + "/moving_speed", moving_speed))
                {
                    ROS_ERROR_STREAM("No moving_speed list for" << operator_names[i]);
                    success = false;
                }
                
                int timeout;
                if (!nh.getParam("/dynamixel_operator/" + operator_names[i] + "/timeout", timeout))
                {
                    ROS_ERROR_STREAM("No timeout for" << operator_names[i]);
                    success = false;
                }

                /*Create New Operator and add into operators*/
                if(success){
                    Operator* to_goal_op = new ToGoalOperator(target_state_publisher, id_list, goal_list, moving_speed, timeout);
                    operator_map[operator_names[i]] = to_goal_op;
                    services.push_back(nh.advertiseService("/head/operator/" + operator_names[i], &Operator::serviceCallback, to_goal_op));
                    ROS_INFO_STREAM("/head/operator/" << operator_names[i] << " service initiated");
                }
            }
            else if (operator_types[i].compare("single") == 0)
            {
                std::string id;
                if (!nh.getParam("/dynamixel_operator/" + operator_names[i] + "/id", id))
                {
                    ROS_ERROR_STREAM("No id for" << operator_names[i]);
                    success = false;
                }
                double cw_lim_value;
                if (!nh.getParam("/dynamixel_operator/" + operator_names[i] + "/cw_lim_value", cw_lim_value))
                {
                    ROS_ERROR_STREAM("No cw_lim_value for" << operator_names[i]);
                    success = false;
                }
                double ccw_lim_value;
                if (!nh.getParam("/dynamixel_operator/" + operator_names[i] + "/ccw_lim_value", ccw_lim_value))
                {
                    ROS_ERROR_STREAM("No ccw_lim_value for" << operator_names[i]);
                    success = false;
                }
                int sleep_time_millis;
                if (!nh.getParam("/dynamixel_operator/" + operator_names[i] + "/sleep_time_millis", sleep_time_millis))
                {
                    ROS_ERROR_STREAM("No sleep_time_millis for" << operator_names[i]);
                    success = false;
                }
                double moving_speed;
                if (!nh.getParam("/dynamixel_operator/" + operator_names[i] + "/moving_speed", moving_speed))
                {
                    ROS_ERROR_STREAM("No moving_speed for" << operator_names[i]);
                    success = false;
                }
                if(success){
                    Operator* single_joint_op  = new SingleJointOperator(nh, target_state_publisher, id, cw_lim_value, ccw_lim_value, sleep_time_millis, moving_speed);
                    operator_map[operator_names[i]] = single_joint_op;
                    services.push_back(nh.advertiseService("/head/operator/" + operator_names[i], &Operator::serviceCallback, single_joint_op));
                    ROS_INFO_STREAM("/head/operator/" << operator_names[i] << " service initiated");
                }
            }
            else if (operator_types[i].compare("sequence") == 0)
            {
                std::vector<std::string> pattern;
                if (!nh.getParam("/dynamixel_operator/" + operator_names[i] + "/pattern", pattern))
                {
                    ROS_ERROR_STREAM("No Pattern for" << operator_names[i]);
                    success = false;
                }
                std::vector<ToGoalOperator*> to_goal_operators;
                for(auto operator_name: pattern){
                    to_goal_operators.push_back(static_cast<ToGoalOperator*>(operator_map[operator_name]));
                }

                if(success){
                    Operator* sequence_op  = new SequenceOperator(nh, target_state_publisher, to_goal_operators);
                    operator_map[operator_names[i]] = sequence_op;
                    services.push_back(nh.advertiseService("/head/operator/" + operator_names[i], &Operator::serviceCallback, sequence_op));
                    ROS_INFO_STREAM("/head/operator/" << operator_names[i] << " service initiated");
                }
            }
            else
            {
                ROS_ERROR_STREAM("Unknown Operator Name " << operator_types[i]);
                success = false;
            }
        }

        return success;
    }
} // namespace dynamixel_tcp