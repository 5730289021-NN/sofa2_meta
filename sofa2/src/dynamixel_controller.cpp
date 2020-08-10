#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include <async_comm/tcp_client.h>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

class Motor
{
public:
	Motor(int id, int cw_lim, int ccw_lim, int inv, int scale): id(id), cw_lim(cw_lim), ccw_lim(ccw_lim), inv(inv), scale(scale), cur_power(0){
		cur_pos = -id;
	}
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
	std::string getStringID() const{
		return std::to_string(id);
	}
	int getPosition() const{
		return cur_pos;
	}
	void setPower(double power) {
		cur_power = power;
		if(power == 0) return;
		if(cur_power > 1) cur_power = 1;
		else if(cur_power < -1) cur_power = -1;
		/*Check constraint*/
		if(cur_pos < cw_lim || cur_pos > ccw_lim) {
			ROS_ERROR_STREAM("For safety purpose, set power = 0 for ID: " << id << " cw_lim: " << cw_lim << " ccw_lim: " << ccw_lim << " pos: " << cur_pos);
			cur_power = 0;
		}
	}
	
	uint8_t getDriveByteLow() const {
		return int(std::abs(cur_power) * scale) % 256;
	}

	uint8_t getDriveByteHigh() const {
		uint8_t hb = std::abs(cur_power) * scale / 256;
		if(cur_power * inv < 0) {
			hb += 4;
		}
		return hb;
	}

	void printMotorRecord(bool printInfo=false) const{
		if(printInfo)
			ROS_INFO_STREAM("Record ID: " << id << " Position: " << cur_pos <<" Address: "<< this);
		else
			ROS_DEBUG_STREAM("Record ID: " << id << " Position: " << cur_pos <<" Address: "<< this);
	}

private:
	int id;
	int cw_lim;
	int ccw_lim;
	int inv;
	int cur_pos;
	int scale;
	double cur_power;
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

	std::vector<uint8_t> tcpMessage;



	int enb_index;

	std::vector<int> id_list;
    std::vector<int> cw_lim_list;
	std::vector<int> ccw_lim_list;
    std::vector<int> inv_list;
	std::vector<double> scale_mul_list;

	std::string ip_addr;
	int port;
	int spin_rate;

	std::vector<Motor*> motors;

	async_comm::TCPClient *tcp_client;

	void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
	void tcpCallback(const uint8_t *buf, size_t len);
	void processMotorMessage(MotorMessage& mm);
	void getReadingCommand(std::vector<uint8_t>& buf) const;
	void getWritingCommand(std::vector<uint8_t>& buf) const;
	sensor_msgs::JointState getJointState() const;
	void fetchRosParameters();
};

void DynamixelController::tcpCallback(const uint8_t *buf, size_t len)
{
	//std::stringstream ss;
	//ss << std::hex;
	// enum StateList {ZERO, HEADER1, HEADER2, ID, LENGTH, ERROR, P1, P2, CHKSUM, LOST};
	// StateList state;
	// StateList prevState;
	// MotorMessage mm;
	for (size_t i = 0; i< len; i++)
	{
		std::cout << std::hex << (int) buf[i];
		std::cout << " ";
		// switch(state) {
		// 	case ZERO:
		// 		if(buf[i] == 0) break;
		// 		else if (buf[i] == 255) state = HEADER1;
		// 		else {
		// 			state = LOST;
		// 			prevState = ZERO;
		// 		}
		// 		break;
		// 	case HEADER1:
		// 		if(buf[i] == 255) state = HEADER2; 
		// 		else {
		// 			/*By pass error in case of 1 ff
		// 			state = LOST;
		// 			prevState = HEADER1;*/
		// 			ROS_WARN("Bypass when there's only one ff");
		// 			mm.id = buf[i]; state = ID;
		// 		}
		// 		break;
		// 	case HEADER2:
		// 		mm.id = buf[i]; state = ID;
		// 		break;
		// 	case ID:
		// 		if(buf[i] == 4) {
		// 			mm.len = buf[i];
		// 			state = LENGTH;
		// 		} else {
		// 			state = LOST;
		// 			prevState = ID;
		// 		}
		// 		break;
		// 	case LENGTH:
		// 		mm.err = buf[i];
		// 		state = ERROR;
		// 		break;
		// 	case ERROR:
		// 		mm.p1 = buf[i]; state = P1;
		// 		break;
		// 	case P1:
		// 		mm.p2 = buf[i]; state = P2;
		// 		break;
		// 	case P2:
		// 		mm.chksum = buf[i]; state = ZERO;
		// 		processMotorMessage(mm);
		// 		break;
		// 	// case CHKSUM:
		// 	// 	mm.chksum = buf[i];
		// 	// 	break;
		// 	default:
		// 		ROS_ERROR("Error while reading Dynamixel message");
		// 		break;
		// }
		// if(state == LOST) {
		// 	//Analyze Error before break
		// 	//std::cout << std::endl;
		// 	ROS_ERROR_STREAM("State Lost at State :" << prevState << " with Packet: ");
		// 	// for(int j = 0; j < len; j++) {
		// 	// 	std::cout << std::hex << (int) buf[j];
		// 	// 	std::cout << " ";
		// 	// }
		// 	//std::cout << std::endl << std::endl;
		// 	break;
		// }
		// if(i == len-1 && state != ZERO) {
		//  	ROS_ERROR_STREAM("Packet Stopped at state: " << state);
		// }
	}
	std::cout << std::endl;
	//ROS_INFO_STREAM("Packet ended with: " << (int) buf[len-1] << " len: " << len);
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

	// for(auto motor:motors){
	// 	motor->printMotorRecord();
	// }

	// 2. Collect p1 p2 to 'id'
	for(auto motor:motors){
		if(motor->getID() == mm.id){
			motor->updatePosition(mm.p2 * 256 + mm.p1);
			motor->printMotorRecord();
			return;
		}
	}
	ROS_ERROR_STREAM("Unknown ID " << mm.id);
}


/*Here's limit this Package functionality*/
/*
	axes[4] -> motors[2]
	axes[3] -> motors[1]
	axes[2] -> motors[0] Down
	axes[5] -> motors[0] Up
*/
void DynamixelController::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	if(msg->buttons[enb_index] == 1) {
		motors[0]->setPower(0.5 * (msg->axes[2] - msg->axes[5]));
		motors[1]->setPower(msg->axes[3]);
		motors[2]->setPower(msg->axes[4]);
	} else {
		motors[0]->setPower(0);
		motors[1]->setPower(0);
		motors[2]->setPower(0);
	}
}

void DynamixelController::getReadingCommand(std::vector<uint8_t>& buf) const
{
	buf = {0xFF, 0xFF, 0xFE, uint8_t(3 * motors.size() + 3), 0x92, 0x00}; // 0xFF, 0xFF, ALL , LEN, INST, 0x00,
	uint8_t chksum = 0xFE + uint8_t(3 * motors.size() + 3) + 0x92;
	for(auto motor:motors) {
		buf.push_back(0x02);
		buf.push_back(motor->getID());
		buf.push_back(0x24);
		chksum += 0x26 + motor->getID();
	}
	chksum = 255 - chksum;
	buf.push_back(chksum);
}

void DynamixelController::getWritingCommand(std::vector<uint8_t>& buf) const
{
	buf = {0xFF, 0xFF, 0xFE, uint8_t(3 * motors.size() + 4), 0x83, 0x32, 0x02}; // 0xFF, 0xFF, ALL , LEN, INST, MOVING_SPEED, 2 BYTE_WRITE
	uint8_t chksum = 0xFE + uint8_t(3 * motors.size() + 4) + 0x83 + 0x32 + 0x02;
	for(auto motor:motors) {

		buf.push_back(motor->getID());
		buf.push_back(motor->getDriveByteLow());
		buf.push_back(motor->getDriveByteHigh());
		chksum += 0x26 + *(buf.end() - 1) + *(buf.end() - 2) + *(buf.end() - 3);
	}
	chksum = 255 - chksum;
	buf.push_back(chksum);
}

void DynamixelController::fetchRosParameters()
{
	nh.param<std::string>("dynamixel_controller/ip_addr", ip_addr, "192.168.16.23");
	nh.param("dynamixel_controller/port", port, 5002);
	nh.param("dynamixel_controller/spin_rate", spin_rate, 5);
	nh.param("dynamixel_controller/enb_index", enb_index, 6);

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
	if (!nh.getParam("dynamixel_controller/scale_mul", scale_mul_list))
	{
		ROS_ERROR("No Dynamixel Scaler Given!");
	}
	int m = id_list.size();
	if(m != cw_lim_list.size() || m != ccw_lim_list.size() || m != ccw_lim_list.size() || m != inv_list.size() || m != scale_mul_list.size()) {
		ROS_ERROR("Dynamixel Parameter Size Mismatched");
	}
}

DynamixelController::DynamixelController()
{
	dynamixelJointPublisher = nh.advertise<sensor_msgs::JointState>("dynamixel_jointstate", 1);
	joySubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 3, &DynamixelController::joyCallback, this);

	fetchRosParameters();

	for(int i = 0; i < id_list.size(); i++){
		Motor* m = new Motor(id_list[i], cw_lim_list[i], ccw_lim_list[i], inv_list[i], scale_mul_list[i] * (ccw_lim_list[i] - cw_lim_list[i]));
		motors.push_back(m);
		//ROS_INFO_STREAM(m->getInfo()); //ROS_INFO requires c_str() probably
	}

	tcp_client = new async_comm::TCPClient(ip_addr, port);
	tcp_client->register_receive_callback(std::bind(&DynamixelController::tcpCallback, this, std::placeholders::_1, std::placeholders::_2));
	
	if (!tcp_client->init())
	{
		ROS_ERROR("Failed to initialize TCP client");
	}

	ros::Rate loop_rate(spin_rate);

	while (ros::ok()) {
		/*TCP Read Current Position*/
		std::vector<uint8_t> buf_read = {0xFF, 0xFF, 0xFE, 0x0C, 0x92, 0x00, 0x02, 0x00, 0x24, 0x02, 0x01, 0x24, 0x02, 0x02, 0x24, 0xEE};
		 // 							0xFF, 0xFF, ALL , LEN, INST, 0x00,
		//getReadingCommand(buf_read);
		ROS_INFO_STREAM("Sending New Package");
		tcp_client->send_bytes(buf_read.data(), buf_read.size());
		/*TCP Write Moving Speed*/
		//std::vector<uint8_t> buf_write;
		//getWritingCommand(buf_write);
		//tcp_client->send_bytes(buf_write.data(), buf_write.size());
		/*Publish ROS Message*/
		sensor_msgs::JointState js;
		for(auto motor:motors)
		{
			//motor->printMotorRecord();
			js.header.stamp = ros::Time::now();
			js.name.push_back(motor->getStringID());
			js.position.push_back(motor->getPosition());
		}
		dynamixelJointPublisher.publish(js);
        ros::spinOnce();
        loop_rate.sleep();
    }
	tcp_client->close();
}

DynamixelController::~DynamixelController()
{
	ROS_INFO("Destructor Called");
	delete tcp_client;
	for(auto m:motors) {
		m->~Motor();
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