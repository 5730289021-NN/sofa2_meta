#include <ros/ros.h>
#include "dynamixel_operator/RosDynamixelOperator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_operator");
  ros::NodeHandle nodeHandle("~");

  dynamixel_tcp::RosDynamixelAdapter ros_dynamixel_operator(nodeHandle);
  
  return 0;
}