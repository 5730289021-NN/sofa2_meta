//
// Created by Brad Bazemore on 10/29/15.
//
// To beable to shutdown, sudo chmod u+s /bin/systemctl
//
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/BatteryState.h>
#include <modbus/modbus.h>
#include <map>
#include <cstdlib>
#include <chrono>
#include <thread>

class plc_modbus_manager
{
public:
    plc_modbus_manager();

private:
    ros::NodeHandle node;

    ros::Subscriber led_head;
    ros::Subscriber led_tray1;

    ros::Publisher battery_publisher;
    sensor_msgs::BatteryState battery_message;

    ros::ServiceServer display_lift_service;
    ros::ServiceServer camera_service;

    ros::ServiceServer shutdown_service;

    static const int coil_size = 5;
    static const std::string coil_name[coil_size];
    uint8_t coil_buffer[coil_size]; /*Coil No.4-8*/

    static const int coil_2_size = 2;
    static const std::string coil_2_name[coil_2_size];
    uint8_t coil_2_buffer[coil_2_size]; /*Coil No. 31-32*/

    static const int holding_reg_size = 10;
    static const std::string holding_reg_name[holding_reg_size];
    uint16_t holding_reg_buffer[holding_reg_size]; /*Holding Register No. 0-9*/

    static const int input_reg_size = 4;
    static const std::string input_reg_name[input_reg_size];
    uint16_t input_reg_buffer[input_reg_size]; /*Input Register No. 20-23*/

    int heartbeat_count;

    std::map<std::string, int> modbus_map;

    modbus_t *plc;

    std::string ip_address;
    std::string plc_type;
    int port;
    int spin_rate;

    void initialize_plc();
    bool modbus_read_value();
    void perform_shutdown();

    uint16_t get_plc_address(uint16_t rockwell_address, uint8_t no_bit);
    void led_head_callback(const std_msgs::ColorRGBA::ConstPtr &led_head_data);
    void led_tray1_callback(const std_msgs::ColorRGBA::ConstPtr &led_tray1_data);
    bool display_lift_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool camera_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool shutdown_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    int get_color_code(const std_msgs::ColorRGBA::ConstPtr &color_data);
};

const std::string plc_modbus_manager::coil_name[coil_size] = {"shutdown_sw", "com_shutdown", "heartbeat", "camera_onoff", "enb_charging"};
const std::string plc_modbus_manager::coil_2_name[coil_2_size] = {"display_up", "display_down"};
const std::string plc_modbus_manager::holding_reg_name[holding_reg_size] = {"head_rgb", "head_time", "tray1_rgb", "tray1_time", "tray2_rgb", "tray2_time",
                                                                            "tray3_rgb", "tray3_time", "sw_bumper", "plc_status"};
const std::string plc_modbus_manager::input_reg_name[input_reg_size] = {"current", "voltage", "capacity_rem", "capacity_tot"};

void plc_modbus_manager::initialize_plc()
{
    ROS_INFO("Connecting to modbus device on %s/%d", ip_address.c_str(), port);
    plc = modbus_new_tcp(ip_address.c_str(), port);
    if (plc == NULL)
    {
        ROS_ERROR("Unable to allocate libmodbus context\n");
        node.setParam("/plc/conn_status", "UNALLOCATED");
        return;
    }
    if (modbus_connect(plc) == -1)
    {
        ROS_ERROR("Failed to connect to modbus device!!!");
        ROS_ERROR("%s", modbus_strerror(errno));
        node.setParam("/plc/conn_status", "DISCONNECTED");
        modbus_free(plc);
        return;
    }
    else
    {
        ROS_INFO("Connection to modbus device established");
        node.setParam("/plc/conn_status", "CONNECTED");
    }

    if (modbus_set_error_recovery(plc, MODBUS_ERROR_RECOVERY_LINK) == -1)
    {
        ROS_ERROR("Unable to set error recovery link");
        node.setParam("/plc/recovery_status", "BAD");
    }
    else
    {
        node.setParam("/plc/recovery_status", "NORMAL");
    }
}

bool plc_modbus_manager::modbus_read_value()
{
    bool success = true;
    if (modbus_read_bits(plc, get_plc_address(4, 1), coil_size, coil_buffer) == -1)
    {
        ROS_ERROR("Error while reading coil_1");
        ROS_ERROR("%s", modbus_strerror(errno));
        success = false;
    }
    else
    {
        for (int i = 0; i < coil_size; i++)
        {
            modbus_map[coil_name[i]] = coil_buffer[i];
            //ROS_INFO_STREAM("Coil Name " << coil_name[i] << " Value : " << (int) coil_buffer[i]);
        }
    }

    if (modbus_read_bits(plc, get_plc_address(31, 1), coil_2_size, coil_2_buffer) == -1)
    {
        ROS_ERROR("Error while reading coil_1");
        ROS_ERROR("%s", modbus_strerror(errno));
        success = false;
    }
    else
    {
        for (int i = 0; i < coil_2_size; i++)
        {
            modbus_map[coil_2_name[i]] = coil_2_buffer[i];
        }
    }

    if (modbus_read_registers(plc, get_plc_address(0, 8), 10, holding_reg_buffer) == -1)
    {
        ROS_ERROR("Error while reading holding register");
        ROS_ERROR("%s", modbus_strerror(errno));
        success = false;
    }
    else
    {
        for (int i = 0; i < holding_reg_size; i++)
        {
            modbus_map[holding_reg_name[i]] = holding_reg_buffer[i];
        }
    }

    /*ROCKWELL has input register while MITSUBISHI hasn't*/
    if(plc_type.compare("ROCKWELL") == 0)
    {
        if (modbus_read_input_registers(plc, get_plc_address(20, 8), 4, input_reg_buffer) == -1)
        {
            ROS_ERROR("Error while reading input register: %d + %d", get_plc_address(20, 8), 4);
            ROS_ERROR("%s", modbus_strerror(errno));
            success = false;
        }
        else
        {
            for (int i = 0; i < input_reg_size; i++)
            {
                modbus_map[input_reg_name[i]] = input_reg_buffer[i];
            }
        }
        break;
    }
    else if(plc_type.compare("MITSUBISHI") == 0)
    {
        if (modbus_read_registers(plc, get_plc_address(20, 8), 4, input_reg_buffer) == -1)
        {
            ROS_ERROR("Error while reading input register: %d + %d", get_plc_address(20, 8), 4);
            ROS_ERROR("%s", modbus_strerror(errno));
            success = false;
        }
        else
        {
            for (int i = 0; i < input_reg_size; i++)
            {
                modbus_map[input_reg_name[i]] = input_reg_buffer[i];
            }
        }
        break;
    }
    return success;
}
/*
rockwell_address is the default address value therefore, we may return input instantly if plc_type is rockwell

no_bit has 2 possible state 1, 8
It's 1 when consider Input Status or Coil
It's 8 when consider Holding Register or Input Register
*/
uint16_t plc_modbus_manager::get_plc_address(uint16_t rockwell_address, uint8_t no_bit)
{
    if (plc_type.compare("ROCKWELL") == 0)
    {
        return rockwell_address;
    }
    else if (plc_type.compare("MITSUBISHI") == 0)
    {
        switch (no_bit)
        {
        case 1:
            return 8192 + rockwell_address;
        case 8:
            return 50 + rockwell_address;
        default:
            ROS_ERROR("Incorrect no_bit value, the no_bit value can be only 1 or 8");
            return rockwell_address;
        }
    }
    else
    {
        ROS_ERROR_STREAM("Unknown PLC_TYPE " << plc_type);
    }
}

bool plc_modbus_manager::shutdown_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    /*req: true -> shutdown*/
    /*req: false -> restart*/
    if (req.data == true)
    {
        /*Shutdown whole system*/
        for (int i = 5; i > 0; i--)
        {
            if (modbus_write_bit(plc, get_plc_address(5, 1), 1) != -1)
            {
                break;
            }
            else
            {
                ROS_ERROR_STREAM("Unable to call PLC shutdown command" << i);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        /*Call Android to shutdown*/
        std::stringstream ss;
        std::string android_ip;
        node.param<std::string>("/android/ip", android_ip, "192.168.16.19");
        ss << "adb connect " << android_ip << "; sleep 3; adb shell reboot -p";

        for (int i = 3; i > 0; i--)
        {
            ROS_INFO_STREAM("Turning off android in..." << i << " using " << ss.str().c_str());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        system(ss.str().c_str());

        /*Self shutdown*/
        for (int i = 5; i > 0; i--)
        {
            ROS_INFO_STREAM("Shutting down(srv) in..." << i);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        //system("sudo shutdown -h now");
        system("systemctl --force --force poweroff");
    }
    else
    {
        /*Call Android to reboot*/
        std::stringstream ss;
        std::string android_ip;
        node.param<std::string>("/android/ip", android_ip, "192.168.16.19");
        ss << "adb connect " << android_ip << "; sleep 3; adb shell reboot";

        for (int i = 3; i > 0; i--)
        {
            ROS_INFO_STREAM("Turning off android in..." << i << " using " << ss.str().c_str());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        system(ss.str().c_str());
        /*Restart only Intel NUC*/
        for (int i = 5; i > 0; i--)
        {
            ROS_INFO_STREAM("Rebooting in..." << i);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        //system("sudo shutdown -r now");
        system("systemctl --force --force reboot");
    }
}

plc_modbus_manager::plc_modbus_manager()
{
    ROS_INFO("PLC Modbus Started");

    heartbeat_count = 0;
    modbus_map["plc_status"] = 99;

    /*Subscriber*/
    led_head = node.subscribe<std_msgs::ColorRGBA>("led/head", 100, &plc_modbus_manager::led_head_callback, this);
    led_tray1 = node.subscribe<std_msgs::ColorRGBA>("led/tray1", 100, &plc_modbus_manager::led_tray1_callback, this);

    /*Publisher*/
    battery_publisher = node.advertise<sensor_msgs::BatteryState>("battery", 1); //TODO
    battery_message.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIMN;
    battery_message.present = true;

    /*Service*/
    display_lift_service = node.advertiseService("display/lift", &plc_modbus_manager::display_lift_callback, this);
    camera_service = node.advertiseService("camera/enable", &plc_modbus_manager::camera_callback, this);
    shutdown_service = node.advertiseService("system/shutdown", &plc_modbus_manager::shutdown_callback, this);

    node.param<std::string>("plc_modbus_node/ip", ip_address, "192.168.16.22");
    node.param<std::string>("plc_modbus_node/plc_type", plc_type, "ROCKWELL");
    node.param("plc_modbus_node/port", port, 502);
    node.param("plc_modbus_node/spin_rate", spin_rate, 2);

    initialize_plc();

    ros::Rate loop_rate(spin_rate);
    while (ros::ok())
    {
        if (!modbus_read_value())
        {
            node.param("/plc/read_value", false);
        }
        else
        {
            node.param("/plc/read_value", true);
        }
        /*Coil No.4 Check if shutdown switch is pushed*/
        if (modbus_map["shutdown_sw"] == 1)
            perform_shutdown();

        /*Coil No. 5 Shutdown from Android triggered from service callback*/
        //Handle at shutdown_callback()

        /*Coil No. 6 Heartbeat*/
        if (modbus_map["heartbeat"] == 0)
        {
            if (modbus_write_bit(plc, get_plc_address(6, 1), 1) == -1)
            {
                ROS_WARN_STREAM("Unable to Write Heartbeat");
            }
            else
            {
                node.setParam("/plc/heartbeat", "NORMAL");
            }
            heartbeat_count = 0;
        }
        else
        {
            heartbeat_count++;
            if (heartbeat_count % 5 == 0)
            {
                ROS_ERROR_STREAM("PLC_UNPINGABLE");
                node.setParam("/plc/heartbeat", "ERROR");
            }
        }

        /*Coil No. 7 Camera Status*/
        node.setParam("/camera/status", modbus_map["camera_onoff"] == 1);

        /*Coil No. 8 Auto Charge*/
        //For future

        /*Coil No. 31-32 Up/Down Display*/
        node.setParam("/display/status", modbus_map["display_up"] && !modbus_map["display_down"] ? "UP" : "DOWN");

        /*Holding Register No. 0-7 LED Color*/
        //No need to publish

        /*Holding Register No. 8 SW-Bumper*/
        node.setParam("/plc/bumper", modbus_map["sw_bumper"]);

        /*Holding Register No. 9 PLC Status*/
        node.setParam("/plc/status", modbus_map["plc_status"]);

        /*Input Register No. 20 - 23 Battery*/
        battery_message.voltage = 0.001 * modbus_map["voltage"];
        battery_message.current = 0.001 * static_cast<int16_t>(modbus_map["current"]);
        battery_message.charge = 0.001 * modbus_map["capacity_rem"];
        battery_message.capacity = 0.001 * modbus_map["capacity_tot"];
        battery_message.percentage = battery_message.charge / battery_message.capacity;

        battery_message.header.stamp = ros::Time::now();
        battery_publisher.publish(battery_message);

        ros::spinOnce();
        loop_rate.sleep();
    }

    modbus_close(plc);
    modbus_free(plc);
    return;
}

void plc_modbus_manager::perform_shutdown()
{
    /*Write back to PLC on Shutdown Acknowledge*/
    for (int i = 5; i > 0; i--)
    {
        if (modbus_write_bit(plc, get_plc_address(4, 1), 0) != -1)
        {
            break;
        }
        else
        {
            ROS_ERROR_STREAM("Unable to call write shutdown back..." << i);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    /*Tell android to turnoff codex*/
    ros::ServiceClient client = node.serviceClient<std_srvs::Trigger>("android/codex_off");
    std_srvs::Trigger codex_off_srv;

    for (int i = 5; i > 0; i--)
    {
        if (client.call(codex_off_srv))
        {
            break;
        }
        else
        {
            ROS_ERROR_STREAM("Unable to call turnoff codex service..." << i);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    /*Call Android to shutdown*/
    std::stringstream ss;
    std::string android_ip;
    node.param<std::string>("/android/ip", android_ip, "192.168.16.19");
    ss << "adb connect " << android_ip << "; sleep 3; adb shell reboot -p";

    for (int i = 3; i > 0; i--)
    {
        ROS_INFO_STREAM("Turning off android in..." << i << " using " << ss.str().c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    system(ss.str().c_str());

    /*Write confirm shutdown to PLC*/
    for (int i = 5; i > 0; i--)
    {
        if (modbus_write_bit(plc, get_plc_address(5, 1), 1) != -1)
        {
            break;
        }
        else
        {
            ROS_ERROR_STREAM("Unable to call write confirm shutdown..." << i);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    /*Shutdown self*/
    for (int i = 5; i > 0; i--)
    {
        ROS_INFO_STREAM("Shutting down(plc) in..." << i);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    //system("sudo shutdown -h now"); /*requires to perform sudo chmod a+s /sbin/shutdown*/
    system("systemctl --force --force poweroff");
}

void plc_modbus_manager::led_head_callback(const std_msgs::ColorRGBA::ConstPtr &led_head_data)
{

    int color_code = get_color_code(led_head_data);

    if (modbus_write_register(plc, get_plc_address(0, 8), color_code) == -1)
    {
        ROS_ERROR("Modbus holding reg write failed at addr:%d with value:%u", 0, color_code);
        ROS_ERROR("%s", modbus_strerror(errno));
    }
    else
    {
        ROS_INFO("Modbus holding reg write at addr:%d with value:%u", 0, color_code);
    }

    if (modbus_write_register(plc, get_plc_address(1, 8), led_head_data->a) == -1)
    {
        ROS_ERROR("Modbus holding reg write failed at addr:%d with value:%u", 0, color_code);
        ROS_ERROR("%s", modbus_strerror(errno));
    }
    else
    {
        ROS_INFO("Modbus holding reg write at addr:%d with value:%u", 0, color_code);
    }
}

void plc_modbus_manager::led_tray1_callback(const std_msgs::ColorRGBA::ConstPtr &led_tray1_data)
{

    int color_code = get_color_code(led_tray1_data);

    if (modbus_write_register(plc, get_plc_address(2, 8), color_code) == -1)
    {
        ROS_ERROR("Modbus holding reg write failed at addr:%d with value:%u", 0, color_code);
        ROS_ERROR("%s", modbus_strerror(errno));
    }
    else
    {
        ROS_INFO("Modbus holding reg write at addr:%d with value:%u", 0, color_code);
    }

    if (modbus_write_register(plc, get_plc_address(3, 8), led_tray1_data->a) == -1)
    {
        ROS_ERROR("Modbus holding reg write failed at addr:%d with value:%u", 0, color_code);
        ROS_ERROR("%s", modbus_strerror(errno));
    }
    else
    {
        ROS_INFO("Modbus holding reg write at addr:%d with value:%u", 0, color_code);
    }
}

bool plc_modbus_manager::display_lift_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    uint8_t plc_tray[2] = {false, false};
    if (req.data)
    {
        plc_tray[0] = true;
        res.message = "Lifted";
        ROS_INFO("Display Lifted");
    }
    else
    {
        plc_tray[1] = true;
        res.message = "Unlifted";
        ROS_INFO("Display Unlifted");
    }
    if (modbus_write_bits(plc, get_plc_address(31, 1), 2, plc_tray) == -1)
    {
        ROS_ERROR("Modbus holding reg write failed at lifting tray");
        ROS_ERROR("%s", modbus_strerror(errno));
        res.success = false;
        res.message = "Failed";
    }
    else
    {
        res.success = true;
    }

    return true;
}

bool plc_modbus_manager::camera_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if (modbus_write_bit(plc, get_plc_address(7, 1), req.data) == -1)
    {
        ROS_ERROR("Modbus holding reg write failed at enable/disable camera");
        ROS_ERROR("%s", modbus_strerror(errno));
        res.success = false;
        res.message = "Failed";
    }
    else
    {
        res.success = true;
        res.message = "Successfully Turn On/Off Camera";
    }
    return true;
}

int plc_modbus_manager::get_color_code(const std_msgs::ColorRGBA::ConstPtr &color_data)
{
    int color_code = 0;
    if (color_data->r > 0)
    {
        if (color_data->g > 0)
        {
            if (color_data->b > 0)
            {
                color_code = 7;
            }
            else
            {
                color_code = 4;
            }
        }
        else if (color_data->b > 0)
        {
            color_code = 5;
        }
        else
        {
            color_code = 1;
        }
    }
    else if (color_data->g > 0)
    {
        if (color_data->b > 0)
        {
            color_code = 6;
        }
        else
        {
            color_code = 2;
        }
    }
    else if (color_data->b > 0)
    {
        color_code = 3;
    }
    return color_code;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_plc_modbus");
    plc_modbus_manager mm;
    return 0;
}
