#include "dynamixel_adapter/dynamixel_adapter.hpp"
#include "dynamixel_adapter/dynamixel_motor.hpp"
#include <ros/console.h>
#include <vector>

namespace dynamixel_tcp
{
    DynamixelAdapter::DynamixelAdapter(std::string ip_addr, int port, std::vector<int> id_list, std::vector<int> cw_lim_list, std::vector<int> ccw_lim_list) : error_msg("")
    {
        /*Initialize motors*/
        for (unsigned i = 0; i < id_list.size(); i++)
        {
            DynamixelMotor *m = new DynamixelMotor(id_list[i], cw_lim_list[i], ccw_lim_list[i]);
            motors.push_back(m);
        }
        /*Initialize TCP and assign callback*/
        tcp_client = new async_comm::TCPClient(ip_addr, port);
        tcp_client->register_receive_callback(std::bind(&DynamixelAdapter::tcpCallback, this, std::placeholders::_1, std::placeholders::_2));
        if (!tcp_client->init())
        {
            ROS_ERROR("DYNAMIXEL_FAILED_TCP_INITIALIZE");
            return;
        }
        /*Enable Torque*/
        std::vector<uint8_t> buf_enb_tq = getTorqueEnableCommand();
        tcp_client->send_bytes(buf_enb_tq.data(), buf_enb_tq.size());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        /*Todo: Config Kp*/
    }

    DynamixelAdapter::~DynamixelAdapter()
    {
        delete tcp_client;
        for (auto m : motors)
        {
            m->~DynamixelMotor();
        }
    }

    std::string DynamixelAdapter::getErrorMsg() const
    {
        return error_msg;
    }

    std::vector<double> DynamixelAdapter::readPositions()
    {
        std::vector<double> positions;
        //1. Send Request Position Command
        for (auto motor : motors)
        {
            std::vector<uint8_t> buf_read_pos = getReadPositionCommand(motor->getID());
            protocol_state = READY;
            tcp_client->send_bytes(buf_read_pos.data(), buf_read_pos.size());
            int count = 0;
            while (protocol_state != COMPLETED)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                count++;
                if (count == TCP_READ_TIMEOUT)
                {
                    ROS_ERROR("DYNAMIXEL_TIMEOUT");
                    break;
                }
            }
            positions.push_back(motor->getPosition());
        }
        //2. Return latest Position (if Possible)
        return positions;
    }

    void DynamixelAdapter::writePositions(std::vector<double> positions)
    {
        std::vector<uint8_t> buf_write_pos = {0xFF, 0xFF, 0xFE, uint8_t(3 * motors.size() + 4), 0x83, 0x20, 0x02}; // 0xFF, 0xFF, ALL , LEN, INST, MOVING_SPEED, 2 BYTE_WRITE
        uint8_t chksum = 0xFE + uint8_t(3 * motors.size() + 4) + 0x83 + 0x20 + 0x02;
        for (unsigned i = 0; i < motors.size(); i++)
        {
            buf_write_pos.push_back(motors[i]->getID());
            buf_write_pos.push_back(static_cast<int>(positions[i]) % 256);
            buf_write_pos.push_back(static_cast<int>(positions[i]) / 256);
            chksum += *(buf_write_pos.end() - 1) + *(buf_write_pos.end() - 2) + *(buf_write_pos.end() - 3);
        }
        chksum = 255 - chksum;
        buf_write_pos.push_back(chksum);

        tcp_client->send_bytes(buf_write_pos.data(), buf_write_pos.size());
    }

    void DynamixelAdapter::tcpCallback(const uint8_t *buf, size_t len)
    {
        for (size_t i = 0; i < len; i++)
        {
            switch (protocol_state) //Protocol finished state
            {
            case READY:
                if (buf[i] == 0)
                    break;
                else if (buf[i] == 255)
                    protocol_state = AFTER_HEADER1;
                else
                    protocol_state = LOST;
                break;
            case AFTER_HEADER1:
                if (buf[i] == 255)
                    protocol_state = AFTER_HEADER2;
                else //Assume when there's only 1 header
                {
                    motor_message.id = buf[i];
                    protocol_state = AFTER_ID;
                }
                break;
            case AFTER_HEADER2:
                motor_message.id = buf[i];
                protocol_state = AFTER_ID;
                break;
            case AFTER_ID:
                if (buf[i] == 4)
                {
                    motor_message.len = buf[i];
                    protocol_state = AFTER_LENGTH;
                }
                else
                    protocol_state = LOST;
                break;
            case AFTER_LENGTH:
                motor_message.err = buf[i];
                protocol_state = AFTER_ERROR;
                break;
            case AFTER_ERROR:
                motor_message.p1 = buf[i];
                protocol_state = AFTER_P1;
                break;
            case AFTER_P1:
                motor_message.p2 = buf[i];
                protocol_state = AFTER_P2;
                break;
            case AFTER_P2:
                motor_message.chksum = buf[i];
                protocol_state = PROCESSING;
                processMotorMessage(motor_message);
                break;
            default:
                ROS_ERROR("DYNAMIXEL_UNEXPECTED_DEFAULT_CASE");
                break;
            }
        }
        if (protocol_state == LOST)
        {
            ROS_ERROR("DYNAMIXEL_STATE_LOST");
        }
    }

    void DynamixelAdapter::processMotorMessage(MotorMessage &motor_message)
    {
        // 1. Process Checksum
        unsigned int calcalated_sum = 255 - ((motor_message.id + motor_message.len + motor_message.err + motor_message.p1 + motor_message.p2) % 256);
        if (calcalated_sum != motor_message.chksum)
        {
            ROS_ERROR("DYNAMIXEL_WRONG_CHKSUM");
            return;
        }

        // 2. Collect p1 p2 to 'id'
        for (auto motor : motors)
        {
            if (motor->getID() == motor_message.id)
            {
                int pos = motor_message.p2 * 256 + motor_message.p1;
                if (!motor->updatePosition(pos))
                {
                    ROS_ERROR("DYNAMIXEL_OUT_OF_BOUND");
                }
                protocol_state = COMPLETED;
                return;
            }
        }
        // UNEXPECTED: When motor->getID() never matches motor_message.id
        error_msg = "DYNAMIXEL_UNKNOWN_ID";
    }

    std::vector<uint8_t> DynamixelAdapter::getTorqueEnableCommand() const
    {
        std::vector<uint8_t> buf = {0xFF, 0xFF, 0xFE, uint8_t(2 * motors.size() + 4), 0x83, 0x18, 0x01}; // 0xFF, 0xFF, ALL , LEN, INST, Torque_ENB, 1 BYTE_WRITE
        uint8_t chksum = 0xFE + uint8_t(2 * motors.size() + 4) + 0x83 + 0x18 + 0x01;
        for (auto motor : motors)
        {
            buf.push_back(motor->getID());
            buf.push_back(0x01);
            chksum += 0x01 + motor->getID();
        }
        chksum = 255 - chksum;
        buf.push_back(chksum);
        return buf;
    }

    std::vector<uint8_t> DynamixelAdapter::getReadPositionCommand(uint8_t id) const
    {
        std::vector<uint8_t> buf = {0xFF, 0xFF, id, 0x04, 0x02, 0x24, 0x02, static_cast<uint8_t>(~(0x2C + id))};
        return buf;
    }
} // namespace dynamixel_tcp