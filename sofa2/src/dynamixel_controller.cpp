#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include <async_comm/tcp_client.h>
#include <sstream>
#include <iostream>
#include <string>

class Motor
{
public:
	Motor(int id, int cw_lim, int ccw_lim, int inv): id(id), cw_lim(cw_lim), ccw_lim(ccw_lim), inv(inv){}
	std::string getInfo(){
		std::stringstream ss;
		ss << "Motor ID:" << id << " CW_LIM:" << cw_lim << " CCW_LIM:" << ccw_lim << " INV:" << inv;
		return ss.str();
	}
private:
	int id;
	int cw_lim;
	int ccw_lim;
	int inv;
};

class DynamixelController
{
public:
	DynamixelController();
	~DynamixelController();

private:
	ros::NodeHandle nh;
	ros::Publisher dynamixelJointPublisher;
	ros::Subscriber joySubscriber;
	sensor_msgs::Joy currentJoyMessage;

	std::vector<int> id_list;
    std::vector<int> cw_lim_list;
	std::vector<int> ccw_lim_list;
    std::vector<int> inv_list;

	std::string ip_addr;
	int port;
	int spin_rate;

	std::vector<Motor> motors;

	async_comm::TCPClient *tcp_client;

	void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
	void tcpCallback(const uint8_t *buf, size_t len);

};

void DynamixelController::tcpCallback(const uint8_t *buf, size_t len)
{
	std::stringstream ss;
	ss << std::hex;
	for (size_t i = 0; i < len; i++)
	{
		ss << buf[i];
	}
	ROS_INFO_STREAM("TCP Received: " << ss.str());
	ROS_INFO_STREAM("size: " << len);
}

void DynamixelController::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	//TODO: Implement here
	ROS_INFO("I heard: [s]");
}

DynamixelController::DynamixelController()
{
	dynamixelJointPublisher = nh.advertise<sensor_msgs::JointState>("dynamixel_jointstate", 1);
	joySubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 3, &DynamixelController::joyCallback, this);

	nh.param<std::string>("dynamixel_controller/ip_addr", ip_addr, "192.168.16.23");
	nh.param("dynamixel_controller/port", port, 5002);
	nh.param("dynamixel_controller/spin_rate", spin_rate, 10);

	if (!nh.getParam("dynamixel_controller/id", id_list))
	{
		ROS_ERROR("No Dynamixel ID Given!");
	}
	if (!nh.getParam("dynamixel_controller/cw_lim", cw_lim_list))
	{
		ROS_ERROR("No Dynamixel Clockwise Limit Given!");
	}
	if (!nh.getParam("dynamixel_controller/ccw_lim", ccw_lim_list))
	{
		ROS_ERROR("No Dynamixel Counter-Clockwise Limit Given!");
	}
	if (!nh.getParam("dynamixel_controller/inv", inv_list))
	{
		ROS_ERROR("No Dynamixel Counter-Clockwise Limit Given!");
	}

	for(int i = 0; i < id_list.size(); i++){
		Motor m(id_list[i], cw_lim_list[i], ccw_lim_list[i], inv_list[i]);
		motors.push_back(m);
		ROS_INFO_STREAM(m.getInfo()); //ROS_INFO requires c_str() probably
	}

	tcp_client = new async_comm::TCPClient(ip_addr, port);
	tcp_client->register_receive_callback(std::bind(&DynamixelController::tcpCallback, this, std::placeholders::_1, std::placeholders::_2));
	
	if (!tcp_client->init())
	{
		ROS_ERROR("Failed to initialize TCP client");
	}

	ros::Rate loop_rate(spin_rate);
	std::vector<uint8_t> buf = {0xFF, 0xFF, 0xFE, 0x0C, 0x92, 0x00, 0x02, 0x00, 0x24, 0x02, 0x01, 0x24, 0x02, 0x02, 0x24, 0xEE};
	//                                     , ALL , LEN, INST, CONST, 2BY.R,ID:0, C.POS,2BY.R,ID:1, C.POS,2BY.R,ID:2, C.POS, CKSUM
    //ROS_INFO_STREAM("Sending: " << buf);
	tcp_client->send_bytes(buf.data(), buf.size());
	while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
	tcp_client->close();
}

DynamixelController::~DynamixelController()
{
	delete tcp_client;
	for(auto m:motors) {
		m.~Motor();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dynamixel_controller");
	DynamixelController dc;
	return 0;
}