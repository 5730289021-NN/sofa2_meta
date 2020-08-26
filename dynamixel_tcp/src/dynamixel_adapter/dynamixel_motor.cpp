#include <string>
#include <sstream>

#include "dynamixel_adapter/dynamixel_motor.hpp"

namespace dynamixel_tcp
{
    DynamixelMotor::DynamixelMotor(int id, int cw_lim, int ccw_lim) : id(id), cw_lim(cw_lim), ccw_lim(ccw_lim)
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

} // namespace ros_package_template
