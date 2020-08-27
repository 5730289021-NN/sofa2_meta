#include <string>
#include <sstream>

#include "dynamixel_adapter/dynamixel_motor.hpp"

namespace dynamixel_tcp
{
    DynamixelMotor::DynamixelMotor(int id, int cw_lim, int ccw_lim, int kp, int ki, int kd)
        : id(id), cw_lim(cw_lim), ccw_lim(ccw_lim), kp(kp), ki(ki), kd(kd)
    {
        cur_pos = -id;
    }
    DynamixelMotor::~DynamixelMotor()
    {
    }

    std::string DynamixelMotor::getInfo() const
    {
        std::stringstream ss;
        ss << "Motor ID:" << id << " CW_LIM:" << cw_lim << " CCW_LIM:" << ccw_lim;
        return ss.str();
    }

    int DynamixelMotor::getID() const
    {
        return id;
    }

    int DynamixelMotor::getKp() const
    {
        return kp;
    }

    int DynamixelMotor::getKi() const
    {
        return ki;
    }

    int DynamixelMotor::getKd() const
    {
        return kd;
    }

    int DynamixelMotor::getIDPIDSum() const
    {
        return id + kp + ki + kd;
    }

    std::string DynamixelMotor::getStringID() const
    {
        return std::to_string(id);
    }

    int DynamixelMotor::getPosition() const
    {
        return cur_pos;
    }

    bool DynamixelMotor::updatePosition(int pos)
    {
        cur_pos = pos;
        return pos >= ccw_lim && pos <= cw_lim;
    }

} // namespace dynamixel_tcp
