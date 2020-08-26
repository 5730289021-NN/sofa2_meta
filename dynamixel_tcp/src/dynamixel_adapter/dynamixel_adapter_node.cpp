#include <ros/ros.h>
#include "dynamixel_adapter/RosDynamixelAdapter.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_adapter");
  ros::NodeHandle nodeHandle("~");

  dynamixel_tcp::RosDynamixelAdapter ros_dynamixel_adapter(nodeHandle);
  
  return 0;
}