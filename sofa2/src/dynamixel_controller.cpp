#include "ros/ros.h"
#include "std_msgs/String.h"

#include <async_comm/tcp_client.h>
#include <sstream>

#include <chrono>
#include <thread>
#include <vector>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

void TCPCallback(const uint8_t* buf, size_t len)
{
  std::stringstream ss;
  for (size_t i = 0; i < len; i++)
  {
    ss << buf[i];
  }
  ROS_INFO_STREAM("TCP Received: " << ss.str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_controller");
  ros::NodeHandle n;
  
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  //ros::Rate loop_rate(10);

  async_comm::TCPClient tcp_client("localhost", 10000);
  tcp_client.register_receive_callback(&TCPCallback);

  if (!tcp_client.init())
  {
    ROS_ERROR("Failed to initialize TCP client");
    return 1;
  }

  ROS_INFO("Start Sending Message");
  for (size_t i = 0; i < 10; ++i)
  {
    std::string msg = "hello world " + std::to_string(i) + "!";
    ROS_INFO_STREAM("Sending: " << msg);
    tcp_client.send_bytes((uint8_t*) msg.data(), msg.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  ROS_INFO("Stop Sending Message");
  tcp_client.close();

  // int count = 0;
  // while (ros::ok())
  // {
  //   std_msgs::String msg;
  //   std::stringstream ss;
  //   ss << "hello world " << count;
  //   msg.data = ss.str();

  //   ROS_INFO("%s", msg.data.c_str());

  //   chatter_pub.publish(msg);

  //   ros::spinOnce();

  //   loop_rate.sleep();
  //   ++count;
  // }
  return 0;
}