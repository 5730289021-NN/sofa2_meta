#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/SetBool.h>
#include <modbus/modbus.h>

class PLCObjectHandler {
public:
    PLCObjectHandler();
private:
    ros::Subscriber subscriber;
    ros::Publisher publisher;
}

PLCObjectHandler::PLCObjectHandler() {

}