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
#include <chrono>
#include <thread>


class Motor
{
public:
	Motor(int id, int cw_lim, int ccw_lim, int inv, int scale): id(id), cw_lim(cw_lim), ccw_lim(ccw_lim), inv(inv), scale(scale), cur_power(0), scaled_power(0){
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
		cur_power = inv * power;
		if(cur_power > 1) cur_power = 1;
		else if(cur_power < -1) cur_power = -1;
		/*Check constraint*/
		if(cur_pos < cw_lim && cur_power < 0 || cur_pos > ccw_lim && cur_power > 0) {
			//When trying to go out of bound
			cur_power = 0;
			ROS_WARN_STREAM("Motor " << id << " : " << cur_pos << " not within (" << cw_lim << ", " << ccw_lim << ")");
		}

		scaled_power = std::abs(cur_power) * scale;
		//ROS_DEBUG_STREAM("Scaled Power: " << scaled_power << " for ID: " << id);
	}
	
	uint8_t getDriveByteLow() const {
		//ROS_DEBUG_STREAM("Motor " << getID() << " , Return Byte Low: " << int(scaled_power % 256) << " from Scaled Power : " << scaled_power << " Scale : " << scale << " Power : " << cur_power);
		return scaled_power % 256;
	}

	uint8_t getDriveByteHigh() const {
		uint8_t hb = scaled_power / 256;
		if(cur_power * inv < 0) {
			hb += 4;
		}
		//ROS_DEBUG_STREAM("Motor " << getID() << " , Return Byte High: " << int(hb) << " from Scaled Power : " << scaled_power);
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
	int scaled_power;
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

struct JoyMessage
{
	bool enb;
	double v0;
	double v1;
	double v2;
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
	sensor_msgs::Joy::ConstPtr joyLatched;

	std::vector<uint8_t> tcpMessage;
	JoyMessage jm;

	enum StateList {ZERO, HEADER1, HEADER2, ID, LENGTH, ERROR, P1, P2, CHKSUM, LOST};
	StateList state;
	StateList prevState;
	MotorMessage mm;

	enum TcpState {READING, COMPLETED};
	TcpState tcpState;

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

	void timerCallback(const ros::TimerEvent& event);
	void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
	void tcpCallback(const uint8_t *buf, size_t len);
	void processMotorMessage(MotorMessage& mm);
	// void getReadingCommand(std::vector<uint8_t>& buf) const;
	std::vector<uint8_t> getWritingTorqueEnableCommand() const;
	std::vector<uint8_t> getReadingCommand(uint8_t id) const;
	std::vector<uint8_t> getWritingCommand() const;
	sensor_msgs::JointState getJointState() const;
	void fetchRosParameters();
};

void DynamixelController::tcpCallback(const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++)
	{
		//std::cout << std::hex << (int) buf[i];
		//std::cout << " ";
		switch(state) {
			case ZERO:
				if(buf[i] == 0) break;
				else if(buf[i] == 255) state = HEADER1;
				else {
					state = LOST;
					prevState = ZERO;
				}
				break;
			case HEADER1:
				if(buf[i] == 255) state = HEADER2; 
				else {
					/*By pass error in case of 1 ff
					state = LOST;
					prevState = HEADER1;*/
					//ROS_WARN("Bypass when there's only one ff");
					mm.id = buf[i]; state = ID;
				}
				break;
			case HEADER2:
				mm.id = buf[i]; state = ID;
				break;
			case ID:
				if(buf[i] == 4) {
					mm.len = buf[i];
					state = LENGTH;
				} else {
					state = LOST;
					prevState = ID;
				}
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
			//std::cout << std::endl;
			ROS_ERROR_STREAM("State Lost at State :" << prevState << " with Packet: ");
			// for(int j = 0; j < len; j++) {
			// 	std::cout << std::hex << (int) buf[j];
			// 	std::cout << " ";
			// }
			//std::cout << std::endl << std::endl;
			tcpState = COMPLETED;
			break;
		}
		if(i == len-1 && state != ZERO) {
			//ROS_WARN_STREAM("Packet Seperate at state: " << state);
		 	//ROS_ERROR_STREAM("Packet Stopped at state: " << state);
		}
	}
	//std::cout << std::endl;
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

	// 2. Collect p1 p2 to 'id'
	for(auto motor:motors){
		if(motor->getID() == mm.id){
			motor->updatePosition(mm.p2 * 256 + mm.p1);
			//motor->printMotorRecord();
			tcpState = COMPLETED;
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

void DynamixelController::timerCallback(const ros::TimerEvent& event) {
	if(jm.enb == 1) {
		motors[0]->setPower(jm.v0);
		motors[1]->setPower(jm.v1);
		motors[2]->setPower(jm.v2);
	} else {
		motors[0]->setPower(0);
		motors[1]->setPower(0);
		motors[2]->setPower(0);
	}
}

void DynamixelController::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
	jm.enb = msg->buttons[enb_index];
	jm.v0 = 0.5 * (msg->axes[2] - msg->axes[5]);
	jm.v1 = msg->axes[3];
	jm.v2 = msg->axes[4];
}

std::vector<uint8_t> DynamixelController::getReadingCommand(uint8_t id) const
{
	std::vector<uint8_t> buf = {0xFF, 0xFF, id, 0x04, 0x02, 0x24, 0x02, ~(0x2C + id)};
	return buf;
}

std::vector<uint8_t> DynamixelController::getWritingTorqueEnableCommand() const
{
	std::vector<uint8_t> buf = {0xFF, 0xFF, 0xFE, uint8_t(2 * motors.size() + 4), 0x83, 0x18, 0x01}; // 0xFF, 0xFF, ALL , LEN, INST, Torque_ENB, 1 BYTE_WRITE
	uint8_t chksum = 0xFE + uint8_t(2 * motors.size() + 4) + 0x83 + 0x18 + 0x01;
	for(auto motor:motors) {
		buf.push_back(motor->getID());
		buf.push_back(0x01);
		chksum += 0x01 + motor->getID();
	}
	chksum = 255 - chksum;
	buf.push_back(chksum);
	return buf;
}

std::vector<uint8_t> DynamixelController::getWritingCommand() const
{
	std::vector<uint8_t> buf = {0xFF, 0xFF, 0xFE, uint8_t(3 * motors.size() + 4), 0x83, 0x20, 0x02}; // 0xFF, 0xFF, ALL , LEN, INST, MOVING_SPEED, 2 BYTE_WRITE
	uint8_t chksum = 0xFE + uint8_t(3 * motors.size() + 4) + 0x83 + 0x20 + 0x02;
	for(auto motor:motors) {
		buf.push_back(motor->getID());
		buf.push_back(motor->getDriveByteLow());
		buf.push_back(motor->getDriveByteHigh());
		chksum += *(buf.end() - 1) + *(buf.end() - 2) + *(buf.end() - 3);
	}
	chksum = 255 - chksum;
	buf.push_back(chksum);
	return buf;
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
	/*ROS Publisher & Subscriber*/
	dynamixelJointPublisher = nh.advertise<sensor_msgs::JointState>("dynamixel_jointstate", 1);
	joySubscriber = nh.subscribe<sensor_msgs::Joy>("joy", 3, &DynamixelController::joyCallback, this);

	/*Fetch required parameters*/
	ROS_INFO("Fetching ROS Parameters");
	fetchRosParameters();
	

	/*Initialize all motor varibles*/
	ROS_INFO("Initializing all motor variables");
	for(int i = 0; i < id_list.size(); i++){
		Motor* m = new Motor(id_list[i], cw_lim_list[i], ccw_lim_list[i], inv_list[i], scale_mul_list[i] * (ccw_lim_list[i] - cw_lim_list[i]));
		motors.push_back(m);
		//ROS_INFO_STREAM(m->getInfo()); //ROS_INFO requires c_str() probably
	}

	/*TCP Client which has only 1 callback when read motor status*/
	ROS_INFO("Setting up TCP Client");
	tcp_client = new async_comm::TCPClient(ip_addr, port);
	tcp_client->register_receive_callback(std::bind(&DynamixelController::tcpCallback, this, std::placeholders::_1, std::placeholders::_2));
	if (!tcp_client->init()) ROS_ERROR("Failed to initialize TCP client");
	tcpState = COMPLETED;

	/*Enable Torque*/
	ROS_INFO("Enabling Motor Torque");
	std::vector<uint8_t> buf_enb_wr = getWritingTorqueEnableCommand();
	tcp_client->send_bytes(buf_enb_wr.data(), buf_enb_wr.size());
	std::this_thread::sleep_for(std::chrono::seconds(1));

	ros::Rate loop_rate(spin_rate);
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), &DynamixelController::timerCallback, this);

	/*Loop*/
	ROS_INFO("Entering Loop");
	while (ros::ok()) {

		/*TCP Read Current Position*/
		for(auto motor:motors)
		{
			std::vector<uint8_t> buf_read = getReadingCommand(motor->getID());
			//ROS_INFO_STREAM("Sending New Package");
			tcp_client->send_bytes(buf_read.data(), buf_read.size());
			tcpState = READING;
			int count = 0;
			while(tcpState == READING){
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				count++;
				if(count == 25){
					ROS_ERROR_STREAM("TIMEOUT");
					break;
				}
			}
			//ROS_DEBUG_STREAM("Solved when within " << count << " millisec");
		}

		/*TCP Write Moving Speed*/
		std::vector<uint8_t> buf_write = getWritingCommand();
		
		/*Verify Packet*/
		std::stringstream ss;
		ss << std::hex << std::setfill(' ');
		for(auto b:buf_write) {
			ss << std::setw(3) << static_cast<unsigned>(b);
			//ROS_INFO_STREAM("Writing : " << std::hex << int(b));
		}
		//ROS_INFO_STREAM("Writing : " << ss.str()); 
		/*End Verify Packet*/
		tcp_client->send_bytes(buf_write.data(), buf_write.size());
		
		
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



// void DynamixelController::getReadingCommand(std::vector<uint8_t>& buf) const
// {
// 	buf = {0xFF, 0xFF, 0xFE, uint8_t(3 * motors.size() + 3), 0x92, 0x00}; // 0xFF, 0xFF, ALL , LEN, INST, 0x00,
// 	uint8_t chksum = 0xFE + uint8_t(3 * motors.size() + 3) + 0x92;
// 	for(auto motor:motors) {
// 		buf.push_back(0x02);
// 		buf.push_back(motor->getID());
// 		buf.push_back(0x24);
// 		chksum += 0x26 + motor->getID();
// 	}
// 	chksum = 255 - chksum;
// 	buf.push_back(chksum);
// }


//std::vector<uint8_t> buf_read = {0xFF, 0xFF, 0xFE, 0x0C, 0x92, 0x00, 0x02, 0x00, 0x24, 0x02, 0x01, 0x24, 0x02, 0x02, 0x24, 0xEE};
//	   							   0xFF, 0xFF, ALL , LEN, INST, 0x00,