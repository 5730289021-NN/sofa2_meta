#include "dynamixel_adapter/dynamixel_adapter.hpp"
#include "dynamixel_adapter/dynamixel_motor.hpp"
#include <ros/console.h>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

#define SHOW_INFO

namespace dynamixel_tcp
{
    DynamixelAdapter::DynamixelAdapter(std::string ip_addr, int port, std::vector<int> id_list,
                                       std::vector<int> cw_lim_list, std::vector<int> ccw_lim_list,
                                       std::vector<int> kp_list, std::vector<int> ki_list, std::vector<int> kd_list)
        : error_msg(""), info_msg(""), protocol_state(READY), ip_addr(ip_addr), port(port)
    {
        /*Initialize motors*/
        for (unsigned i = 0; i < id_list.size(); i++)
        {
            DynamixelMotor *m = new DynamixelMotor(id_list[i], cw_lim_list[i], ccw_lim_list[i], kp_list[i], ki_list[i], kd_list[i]);
            motors.push_back(m);
        }
        /*Initialize TCP and assign callback*/
        tcp_client = new async_comm::TCPClient(ip_addr, port);
        if (!tcp_client->init())
        {
            setErrorMsg("DYNAMIXEL_FAILED_TCP_INITIALIZE");
            return;
        }
        /*Config Kp, Ki, Kd*/
        setInfoMsg("Configuring PID");
        std::vector<uint8_t> buf_con_pid = getPIDConfigCommand();
        tcp_client->send_bytes(buf_con_pid.data(), buf_con_pid.size());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        /*Enable Torque*/
        setInfoMsg("Enabling Torque");
        std::vector<uint8_t> buf_enb_tq = getTorqueEnableCommand();
        tcp_client->send_bytes(buf_enb_tq.data(), buf_enb_tq.size());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        setInfoMsg("Assigning TCP Callback");
        tcp_client->register_receive_callback(std::bind(&DynamixelAdapter::tcpCallback, this, std::placeholders::_1, std::placeholders::_2));
        setInfoMsg("Finish assign callback");
    }

    DynamixelAdapter::~DynamixelAdapter()
    {
        setInfoMsg("Dynamixel Adapter Destructor Called");
        tcp_client->close();
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

    /*Expected copy elision*/
    std::vector<double> DynamixelAdapter::readPositions()
    {
        std::vector<double> positions;
        //1. Send Request Position Command
        for (auto motor : motors)
        {
            std::vector<uint8_t> buf_read_pos = getReadPositionCommand(motor->getID());
            protocol_state = READY;
            tcp_client->send_bytes(buf_read_pos.data(), buf_read_pos.size());
            unsigned count = 0;
            while (protocol_state != COMPLETED)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                count++;
                if (count == TCP_READ_TIMEOUT)
                {
                    timeout_stack++;
                    setErrorMsg("DYNAMIXEL_TIMEOUT_ID_" + motor->getStringID() + "_STACKED_" + std::to_string(timeout_stack));
                    break;
                }
            }
            if (count != TCP_READ_TIMEOUT)
            {
                timeout_stack = 0;
            }
            //When protocol_state is COMPLETE then motor position is automically updated
            positions.push_back(motor->getPosition());
        }
        //2. Return latest Position (if Possible)
        return positions;
    }

    void DynamixelAdapter::writePositions(std::vector<std::string> &ids, std::vector<double> &positions)
    {
        std::vector<uint8_t> buf_write_pos = {0xFF, 0xFF, 0xFE, uint8_t(3 * ids.size() + 4), 0x83, 0x1E, 0x02}; // 0xFF, 0xFF, ALL , LEN, INST, POSITION, 2 BYTE_WRITE
        uint8_t chksum = 0xFE + uint8_t(3 * ids.size() + 4) + 0x83 + 0x1E + 0x02;
        for (unsigned i = 0; i < ids.size(); i++)
        {
            buf_write_pos.push_back(static_cast<uint8_t>(std::stoi(ids[i])));
            buf_write_pos.push_back(static_cast<uint8_t>(positions[i]) % 256);
            buf_write_pos.push_back(static_cast<uint8_t>(positions[i] / 256));
            chksum += *(buf_write_pos.end() - 1) + *(buf_write_pos.end() - 2) + *(buf_write_pos.end() - 3);
        }
        chksum = 0xFF - chksum;
        buf_write_pos.push_back(chksum);
        std::stringstream ss;
        ss << std::hex << std::setfill(' ');
        for (int i = 0; i < buf_write_pos.size(); i++)
        {
            ss << std::setw(3) << static_cast<unsigned>(buf_write_pos[i]);
        }
        setInfoMsg(ss.str());

        tcp_client->send_bytes(buf_write_pos.data(), buf_write_pos.size());
    }

    void DynamixelAdapter::writePositionsVelocities(std::vector<std::string> &ids, std::vector<double> &positions, std::vector<double> &velocities)
    {
        std::vector<uint8_t> buf_write_pos = {0xFF, 0xFF, 0xFE, uint8_t(5 * ids.size() + 4), 0x83, 0x1E, 0x04}; // 0xFF, 0xFF, ALL , LEN, INST, POSITION, 4 BYTE_WRITE
        uint8_t chksum = 0xFE + uint8_t(5 * ids.size() + 4) + 0x83 + 0x1E + 0x04;
        /*
        */
        for (unsigned i = 0; i < ids.size(); i++)
        {
            buf_write_pos.push_back(static_cast<uint8_t>(std::stoi(ids[i])));
            if (static_cast<uint16_t>(velocities[i]) == 0)
            {
                /*Override Behavior: When writing zero velocity, just send the current position with any velocity */
                bool motor_found = false;
                for (auto motor : motors)
                {
                    if (motor->getStringID() == ids[i])
                    {
                        setInfoMsg("Sending current Position Instead");
                        setInfoMsg(motor->getInfo());
                        buf_write_pos.push_back(static_cast<uint8_t>(motor->getPosition()) % 256);
                        buf_write_pos.push_back(static_cast<uint8_t>(motor->getPosition() / 256));
                        motor_found = true;
                    }
                }
                if (!motor_found)
                {
                    setErrorMsg("Motor not registered: ID" + ids[i]);
                    buf_write_pos.push_back(static_cast<uint8_t>(positions[i]) % 256);
                    buf_write_pos.push_back(static_cast<uint8_t>(positions[i] / 256));
                }
            }
            else
            {
                buf_write_pos.push_back(static_cast<uint8_t>(positions[i]) % 256);
                buf_write_pos.push_back(static_cast<uint8_t>(positions[i] / 256));
            }
            buf_write_pos.push_back(static_cast<uint8_t>(velocities[i]) % 256);
            buf_write_pos.push_back(static_cast<uint8_t>(velocities[i] / 256));
            chksum += *(buf_write_pos.end() - 1) + *(buf_write_pos.end() - 2) + *(buf_write_pos.end() - 3) + *(buf_write_pos.end() - 4) + *(buf_write_pos.end() - 5);
        }
        chksum = 0xFF - chksum;
        buf_write_pos.push_back(chksum);
        std::stringstream ss;
        ss << std::hex << std::setfill(' ');
        for (int i = 0; i < buf_write_pos.size(); i++)
        {
            ss << std::setw(3) << static_cast<unsigned>(buf_write_pos[i]);
        }
        setInfoMsg(ss.str());
        tcp_client->send_bytes(buf_write_pos.data(), buf_write_pos.size());
    }

    void DynamixelAdapter::tcpCallback(const uint8_t *buf, size_t len)
    {
        // std::cout <<std::hex<< buf;
        // setInfoMsg("Start");
        // std::string s;
        // s.assign(buf, buf + len);
        // setInfoMsg(std::to_string(int(len)));
        // setInfoMsg("End");
        for (size_t i = 0; i < len; i++)
        {
            switch (protocol_state) //Protocol finished state
            {
            case READY:
                //setInfoMsg("At READY");
                if (buf[i] == 0)
                    break;
                else if (buf[i] == 255)
                    protocol_state = AFTER_HEADER1;
                else
                    protocol_state = LOST;
                break;
            case AFTER_HEADER1:
                //setInfoMsg("At AFTER_HEADER1");
                if (buf[i] == 255)
                    protocol_state = AFTER_HEADER2;
                else //Assume when there's only 1 header
                {
                    motor_message.id = buf[i];
                    protocol_state = AFTER_ID;
                }
                break;
            case AFTER_HEADER2:
                //setInfoMsg("At AFTER_HEADER2");
                motor_message.id = buf[i];
                protocol_state = AFTER_ID;
                break;
            case AFTER_ID:
                //setInfoMsg("At AFTER_ID");
                if (buf[i] == 4)
                {
                    motor_message.len = buf[i];
                    protocol_state = AFTER_LENGTH;
                }
                else
                    protocol_state = LOST;
                break;
            case AFTER_LENGTH:
                //setInfoMsg("At AFTER_LENGTH");
                motor_message.err = buf[i];
                protocol_state = AFTER_ERROR;
                break;
            case AFTER_ERROR:
                //setInfoMsg("At AFTER_ERROR");
                motor_message.p1 = buf[i];
                protocol_state = AFTER_P1;
                break;
            case AFTER_P1:
                //setInfoMsg("At AFTER_P1");
                motor_message.p2 = buf[i];
                protocol_state = AFTER_P2;
                break;
            case AFTER_P2:
                //setInfoMsg("At AFTER_P2");
                motor_message.chksum = buf[i];
                protocol_state = PROCESSING;
                processMotorMessage(motor_message);
                break;
            case PROCESSING:
            case COMPLETED:
            {
                // std::string s("Residue: ");
                // s += buf[i];
                // setInfoMsg(s);
                break;
            }
            case LOST:
                setErrorMsg("DYNAMIXEL_STATE_LOST");
                for (int j = 0; j < len; j++)
                {
                    std::cout << std::hex << (int)buf[j];
                    std::cout << " ";
                }
                protocol_state = COMPLETED;
                return;
            default:
                setErrorMsg("DYNAMIXEL_UNEXPECTED_DEFAULT_CASE");
                protocol_state = COMPLETED;
                return;
            }
        }
    }

    void DynamixelAdapter::processMotorMessage(MotorMessage &motor_message)
    {
        // 1. Process Checksum
        unsigned int calcalated_sum = 255 - ((motor_message.id + motor_message.len + motor_message.err + motor_message.p1 + motor_message.p2) % 256);
        if (calcalated_sum != motor_message.chksum)
        {
            setErrorMsg("DYNAMIXEL_WRONG_CHKSUM");
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
                    setWarningMsg("DYNAMIXEL_OUT_OF_BOUND");
                    setWarningMsg(motor->getInfo());
                }
                protocol_state = COMPLETED;
                return;
            }
        }
        // UNEXPECTED: When motor->getID() never matches motor_message.id
        setErrorMsg("DYNAMIXEL_UNKNOWN_ID");
    }

    std::vector<uint8_t> DynamixelAdapter::getTorqueEnableCommand() const
    {
        std::vector<uint8_t> buf = {0xFF, 0xFF, 0xFE, uint8_t(2 * motors.size() + 4), 0x83, 0x18, 0x01}; // 0xFF, 0xFF, ALL , LEN, INST, Torque_ENB, 1 BYTE_WRITE
        uint8_t chksum = 0xFE + uint8_t(2 * motors.size() + 4) + 0x83 + 0x18 + 0x01;
        for (auto motor : motors)
        {
            buf.push_back(motor->getID());
            buf.push_back(0x01); //Enable: 0x01
            chksum += 0x01 + motor->getID();
        }
        chksum = 0xFF - chksum;
        buf.push_back(chksum);
        return buf;
    }

    std::vector<uint8_t> DynamixelAdapter::getPIDConfigCommand() const
    {
        //0xFF, 0xFF, ALL , LEN((3_data+1_con)*3_motor + 4_con), INST(0x83_bulk_write), D Gain(26), 3_BYTE_WRITE
        std::vector<uint8_t> buf = {0xFF, 0xFF, 0xFE, uint8_t(4 * motors.size() + 4), 0x83, 0x1A, 0x03};
        uint8_t chksum = 0xFE + uint8_t(4 * motors.size() + 4) + 0x83 + 0x1A + 0x03;
        for (auto motor : motors)
        {
            buf.push_back(motor->getID());
            buf.push_back(motor->getKd()); //Kd
            buf.push_back(motor->getKi()); //Ki
            buf.push_back(motor->getKp()); //Kp
            chksum += motor->getIDPIDSum();
        }
        chksum = 0xFF - chksum;
        buf.push_back(chksum);
        return buf;
    }

    std::vector<uint8_t> DynamixelAdapter::getReadPositionCommand(uint8_t id) const
    {
        std::vector<uint8_t> buf = {0xFF, 0xFF, id, 0x04, 0x02, 0x24, 0x02, static_cast<uint8_t>(~(0x2C + id))};
        return buf;
    }

    bool DynamixelAdapter::tryReconnect()
    {
        setInfoMsg("Closing for 3 seconds");
        tcp_client->close();
        std::this_thread::sleep_for(std::chrono::seconds(3));
        setInfoMsg("Reconnecting...");
        //delete tcp_client;
        tcp_client = new async_comm::TCPClient(ip_addr, port);
        if (!tcp_client->init())
        {
            setErrorMsg("DYNAMIXEL_FAILED_TCP_RECONNECT");
            return false;
        }
        tcp_client->register_receive_callback(std::bind(&DynamixelAdapter::tcpCallback, this, std::placeholders::_1, std::placeholders::_2));
        //Assume Successfully Connect
        timeout_stack = 0;
        return true;
    }

    bool DynamixelAdapter::getStatus() const
    {
        return (timeout_stack < TCP_STACK_RESET);
    }

    inline void DynamixelAdapter::setErrorMsg(std::string error)
    {
        error_msg = error;
        ROS_ERROR_STREAM(error_msg);
    }

    inline void DynamixelAdapter::setWarningMsg(std::string warning)
    {
        //ROS_WARN_STREAM(warning);
    }

    inline void DynamixelAdapter::setInfoMsg(std::string info)
    {
        info_msg = info;
        ROS_INFO_STREAM(info);
    }
} // namespace dynamixel_tcp