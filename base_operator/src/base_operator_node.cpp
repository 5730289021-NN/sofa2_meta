#include <ros/ros.h>
#include "base_operator/RosBaseOperator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_operator");
  ros::NodeHandle nodeHandle("~");

  base_operator::RosBaseOperator ros_base_operator(nodeHandle);
  
  return 0;
}