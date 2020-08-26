#pragma once

#include <string>

namespace dynamixel_tcp
{
    class DynamixelMotor
    {
    public:
        DynamixelMotor(int id, int cw_lim, int ccw_lim);
        virtual ~DynamixelMotor();

        std::string getInfo() const;
        int getID() const;
        std::string getStringID() const;
        int getPosition() const;
        void getMotorRecord(bool printInfo = false) const;

        bool updatePosition(int pos);

    private:
        int id;
        int cw_lim;
        int ccw_lim;
        int cur_pos;
    };
} // namespace dynamixel_tcp
