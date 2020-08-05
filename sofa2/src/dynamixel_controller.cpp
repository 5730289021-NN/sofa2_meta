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
	void updatePosition(int pos){
		cur_pos = pos;
	}
	int getID() const{
		return id;
	}
	void printMotorRecord() const{
		ROS_INFO_STREAM("Record ID: " << id << " Position: " << cur_pos);
	}
private:
	int id;
	int cw_lim;
	int ccw_lim;
	int inv;
	int cur_pos;
};

struct MotorMessage
{
	uint8_t id;
	uint8_t len;
	uint8_t err;
	unsigned int p1;
	unsigned int p2;
	unsigned int chksum;
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
	void processMotorMessage(MotorMessage& mm);
};

void DynamixelController::tcpCallback(const uint8_t *buf, size_t len)
{
	//std::stringstream ss;
	//ss << std::hex;
	enum StateList {ZERO, HEADER1, HEADER2, ID, LENGTH, ERROR, P1, P2, CHKSUM, LOST};
	StateList state = ZERO;
	MotorMessage mm;
	for (size_t i = 0; i < len; i++)
	{
		std::cout << std::hex << (int) buf[i];
		std::cout << " ";
		switch(state) {
			case ZERO:
				if(buf[i] == 0) break;
				else if (buf[i] == 255) state = HEADER1;
				else state = LOST;
				break;
			case HEADER1:
				if(buf[i] == 255) state = HEADER2; else state = LOST;
				break;
			case HEADER2:
				mm.id = buf[i]; state = ID;
				break;
			case ID:
				if(buf[i] == 4) {
					mm.len = buf[i];
					state = LENGTH;
				} else
					state = LOST;
				break;
			case LENGTH:
				mm.err = buf[i];
				state = ERROR;
				break;
			case ERROR:
				mm.p1 = buf[i]; state = P1;
				break;
			case P1:
				mm.p2 = buf[i]; state = P2;
				break;
			case P2:
				mm.chksum = buf[i]; state = ZERO;
				processMotorMessage(mm);
				break;
			// case CHKSUM:
			// 	mm.chksum = buf[i];
			// 	break;
			default:
				ROS_ERROR("Error while reading Dynamixel message");
				break;
		}
		if(state == LOST) {
			//Analyze Error before break
			std::cout << std::endl;
			ROS_ERROR_STREAM("State Lost from Packet :");
			for(int j = 0; j < len; j++) {
				std::cout << std::hex << (int) buf[j];
				std::cout << " ";
			}
			std::cout << std::endl << std::endl;
			break;
		}
	}
	/*std::cout << std::endl;
	ROS_INFO_STREAM("Packet ended with: " << (int) buf[len-1] << " len: " << len);
	*/
}

void DynamixelController::processMotorMessage(MotorMessage& mm)
{
	// 1. Process Checksum
	unsigned int calsum = 255 - ((mm.id + mm.len + mm.err + mm.p1 + mm.p2) % 256);
	if(calsum != mm.chksum)
	{
		ROS_ERROR_STREAM("Wrong Checksum Calculated: " << calsum << " Actual Sum: " << mm.chksum);
		return;
	}
	// 2. Collect p1 p2 to 'id'
	for(auto m:motors){
		if(m.getID() == mm.id){
			m.updatePosition(mm.p2 * 256 + mm.p1);
			m.printMotorRecord();
			return;
		}
	}
	ROS_ERROR_STREAM("Unknown ID " << mm.id);
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
	while (ros::ok()) {
		tcp_client->send_bytes(buf.data(), buf.size());
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
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
	ros::init(argc, argv, "dynamixel_controller");
	DynamixelController dc;
	return 0;
}