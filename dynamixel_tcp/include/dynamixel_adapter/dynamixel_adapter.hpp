#pragma once

#include <dynamixel_adapter/dynamixel_motor.hpp>
#include <vector>
#include <string>
#include <async_comm/tcp_client.h>

namespace dynamixel_tcp
{
    enum DynamixelProtocolState
    {
        READY,
        AFTER_HEADER1,
        AFTER_HEADER2,
        AFTER_ID,
        AFTER_LENGTH,
        AFTER_ERROR,
        AFTER_P1,
        AFTER_P2,
        AFTER_CHKSUM, //Not used
        PROCESSING,
        COMPLETED,
        LOST
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

    class DynamixelAdapter
    {
    public:
        DynamixelAdapter(){};
        DynamixelAdapter(const DynamixelAdapter &rhs);
        DynamixelAdapter(std::string ip_addr, int port, std::vector<int> id_list, std::vector<int> cw_lim_list, std::vector<int> ccw_lim_list,
                         std::vector<int> kp_list, std::vector<int> ki_list, std::vector<int> kd_list);
        virtual ~DynamixelAdapter();
        DynamixelAdapter &operator=(const DynamixelAdapter &other) = default;

        std::string getErrorMsg() const;
        std::vector<double> readPositions(); //Send a request position command and return the that value
        void writePositions(std::vector<std::string> &ids, std::vector<double> &positions);
        void writePositionsVelocities(std::vector<std::string> &ids, std::vector<double> &positions, std::vector<double> &velocities);
        void setVelocities(std::vector<int> velocities);

    private:
        async_comm::TCPClient *tcp_client;
        std::vector<DynamixelMotor *> motors;
        volatile DynamixelProtocolState protocol_state;
        MotorMessage motor_message;

        std::string error_msg;
        std::string info_msg;

        static int const TCP_READ_TIMEOUT = 50;

        int timeout_stack;

        void tcpCallback(const uint8_t *buf, size_t len);
        void processMotorMessage(MotorMessage &motor_message);
        std::vector<uint8_t> getTorqueEnableCommand() const;
        std::vector<uint8_t> getPIDConfigCommand() const;
        std::vector<uint8_t> getReadPositionCommand(uint8_t id) const;
    
        inline void setErrorMsg(std::string error);
        inline void setWarningMsg(std::string warning);
        inline void setInfoMsg(std::string info);
    };

} // namespace dynamixel_tcp