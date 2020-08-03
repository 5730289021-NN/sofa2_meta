//
// Created by Brad Bazemore on 10/29/15.
//
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/SetBool.h>
#include <modbus/modbus.h>

class plc_modbus_manager {
public:
    plc_modbus_manager();

private:
    ros::NodeHandle node;

    ros::Publisher holding_regs_read;
    ros::Subscriber holding_regs_write;
    ros::Publisher coils_read;
    ros::Subscriber coils_write;

    ros::Subscriber led_head;
    ros::Subscriber led_tray1;
    ros::ServiceServer display_lift_service;
    ros::ServiceServer camera_service;

    std::vector<int> holding_regs_addr;
    std::vector<int> coils_addr;

    std_msgs::UInt16MultiArray holding_regs_val;
    std_msgs::ByteMultiArray coils_val;

    modbus_t *plc;

    std::string ip_address;
    int port;
    int spin_rate;

    void holding_regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &holding_regs_data);
    void coils_callBack(const std_msgs::ByteMultiArray::ConstPtr &coils_data);
    void led_head_callBack(const std_msgs::ColorRGBA::ConstPtr &led_head_data);
    void led_tray1_callBack(const std_msgs::ColorRGBA::ConstPtr &led_tray1_data);
    bool display_lift_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool camera_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    int get_color_code(const std_msgs::ColorRGBA::ConstPtr &color_data);

};


plc_modbus_manager::plc_modbus_manager() {

    holding_regs_read = node.advertise<std_msgs::UInt16MultiArray>("modbus/holding_regs_read", 100);
    holding_regs_write = node.subscribe<std_msgs::UInt16MultiArray>("modbus/holding_regs_write", 100, &plc_modbus_manager::holding_regs_callBack, this);
    coils_read = node.advertise<std_msgs::ByteMultiArray>("modbus/coils_read", 100);
    coils_write = node.subscribe<std_msgs::ByteMultiArray>("modbus/coils_write", 100, &plc_modbus_manager::coils_callBack, this);

    led_head = node.subscribe<std_msgs::ColorRGBA>("led/head", 100, &plc_modbus_manager::led_head_callBack, this);
    led_tray1 = node.subscribe<std_msgs::ColorRGBA>("led/tray1", 100, &plc_modbus_manager::led_tray1_callBack, this);

    display_lift_service = node.advertiseService("display/lift", &plc_modbus_manager::display_lift_callback, this);
    camera_service = node.advertiseService("camera/enable", &plc_modbus_manager::camera_callback, this);

    node.param<std::string>("plc_modbus_node/ip", ip_address, "192.168.16.22");
    node.param("plc_modbus_node/port", port, 502);
    node.param("plc_modbus_node/spin_rate",spin_rate,10);

    if (!node.getParam("plc_modbus_node/holding_regs_addr", holding_regs_addr)) {
        ROS_WARN("No reg addrs given!");
    }
    if (!node.getParam("plc_modbus_node/coils_addr", coils_addr)) {
        ROS_WARN("No coil addrs given!");
    }

    ROS_INFO("Connecting to modbus device on %s/%d", ip_address.c_str(), port);
    plc = modbus_new_tcp(ip_address.c_str(), port);
    if (plc == NULL) {
        ROS_FATAL("Unable to allocate libmodbus context\n");
        return;
    }
    if (modbus_connect(plc) == -1) {
        ROS_FATAL("Failed to connect to modbus device!!!");
        ROS_FATAL("%s", modbus_strerror(errno));
        modbus_free(plc);
        return;
    } else {
        ROS_INFO("Connection to modbus device established");
    }

    ros::Rate loop_rate(spin_rate);

    while (ros::ok()) {
        holding_regs_val.data.clear();
        coils_val.data.clear();

        for (int i = 0; i < holding_regs_addr.size(); i++) {
            uint16_t temp[1] = {0};
            if (modbus_read_registers(plc, holding_regs_addr.at(i), 1, temp) == -1) {
                ROS_ERROR("Unable to read reg addr:%d", holding_regs_addr.at(i));
                ROS_ERROR("%s", modbus_strerror(errno));
            } else {
                holding_regs_val.data.push_back(temp[0]);
            }
        }
        if (holding_regs_val.data.size() > 0) {
            holding_regs_read.publish(holding_regs_val);
        }

        for (int i = 0; i < coils_addr.size(); i++) {
            uint8_t temp[1] = {0};
            if (modbus_read_bits(plc, coils_addr.at(i), 1, temp) == -1) {
                ROS_ERROR("Unable to read coil addr:%d", coils_addr.at(i));
                ROS_ERROR("%s", modbus_strerror(errno));
            } else {
                coils_val.data.push_back(temp[0]);
            }
        }
        if (coils_val.data.size() > 0) {
            coils_read.publish(coils_val);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    modbus_close(plc);
    modbus_free(plc);
    return;
}

void plc_modbus_manager::holding_regs_callBack(const std_msgs::UInt16MultiArray::ConstPtr &holding_regs_data) {
    if (holding_regs_data->data.size() != holding_regs_addr.size()) {
        ROS_ERROR("%d holding registers to write but only %d given!", holding_regs_addr.size(), holding_regs_data->data.size());
        return;
    }
    for (int i = 0; i < holding_regs_data->data.size(); i++) {
        ROS_DEBUG("holding regs_out[%d]:%u", i, holding_regs_data->data.at(i));
        uint16_t temp[1] = {holding_regs_data->data.at(i)};
        if (modbus_write_registers(plc, holding_regs_addr.at(i), 1, temp) == -1) {
            ROS_ERROR("Modbus holding reg write failed at addr:%d with value:%u", holding_regs_addr.at(i), holding_regs_data->data.at(i));
            ROS_ERROR("%s", modbus_strerror(errno));
        } else {
            ROS_INFO("Modbus holding register write at addr:%d with value:%u", holding_regs_addr.at(i), holding_regs_data->data.at(i));
        }
    }
}

void plc_modbus_manager::coils_callBack(const std_msgs::ByteMultiArray::ConstPtr &coils_data) {
    if (coils_data->data.size() != coils_addr.size()) {
        ROS_ERROR("%d coils to write but %d given!", coils_addr.size(), coils_data->data.size());
        return;
    }
    for (int i = 0; i < coils_data->data.size(); i++) {
        ROS_DEBUG("regs_out[%d]:%u", i, coils_data->data.at(i));
        uint8_t temp[1] = {coils_data->data.at(i)};
        if (modbus_write_bits(plc, coils_addr.at(i), 1, temp) == -1) {
            ROS_ERROR("Modbus coil write failed at addr:%d with value:%u", coils_addr.at(i), coils_data->data.at(i));
            ROS_ERROR("%s", modbus_strerror(errno));
        } else {
            ROS_INFO("Modbus coil write at addr:%d with value:%u", coils_addr.at(i), coils_data->data.at(i));
        }
    }
}

void plc_modbus_manager::led_head_callBack(const std_msgs::ColorRGBA::ConstPtr &led_head_data) {

    int color_code = get_color_code(led_head_data);
    
    if (modbus_write_register(plc, 0, color_code) == -1) {
        ROS_ERROR("Modbus holding reg write failed at addr:%d with value:%u", 0, color_code);
        ROS_ERROR("%s", modbus_strerror(errno));
    } else {
        ROS_INFO("Modbus holding reg write at addr:%d with value:%u", 0, color_code);
    }

    if (modbus_write_register(plc, 1, led_head_data->a) == -1) {
        ROS_ERROR("Modbus holding reg write failed at addr:%d with value:%u", 0, color_code);
        ROS_ERROR("%s", modbus_strerror(errno));
    } else {
        ROS_INFO("Modbus holding reg write at addr:%d with value:%u", 0, color_code);
    }

}

void plc_modbus_manager::led_tray1_callBack(const std_msgs::ColorRGBA::ConstPtr &led_tray1_data) {

    int color_code = get_color_code(led_tray1_data);
    
    if (modbus_write_register(plc, 2, color_code) == -1) {
        ROS_ERROR("Modbus holding reg write failed at addr:%d with value:%u", 0, color_code);
        ROS_ERROR("%s", modbus_strerror(errno));
    } else {
        ROS_INFO("Modbus holding reg write at addr:%d with value:%u", 0, color_code);
    }

    if (modbus_write_register(plc, 3, led_tray1_data->a) == -1) {
        ROS_ERROR("Modbus holding reg write failed at addr:%d with value:%u", 0, color_code);
        ROS_ERROR("%s", modbus_strerror(errno));
    } else {
        ROS_INFO("Modbus holding reg write at addr:%d with value:%u", 0, color_code);
    }

}

bool plc_modbus_manager::display_lift_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res){
    uint8_t plc_tray[2] = {false, false};
    if (req.data){
        plc_tray[0] = true;         
        res.message = "Lifted";
        ROS_INFO("Display Lifted");
    } else
    {
        plc_tray[1] = true;
        res.message = "Unlifted";
        ROS_INFO("Display Unlifted");
    }
    if(modbus_write_bits(plc, 31, 2, plc_tray) == -1) {
        ROS_ERROR("Modbus holding reg write failed at lifting tray");
        ROS_ERROR("%s", modbus_strerror(errno));
        res.success = false;
        res.message = "Failed";
    } else {
        res.success = true;
    }
    return true;
}

bool plc_modbus_manager::camera_callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res){
    if(modbus_write_bit(plc, 7, req.data) == -1) {
        ROS_ERROR("Modbus holding reg write failed at enbable/disable camera");
        ROS_ERROR("%s", modbus_strerror(errno));
        res.success = false;
        res.message = "Failed";
    } else {
        res.success = true;
    }
    return true;
}

int plc_modbus_manager::get_color_code(const std_msgs::ColorRGBA::ConstPtr &color_data) {
    int color_code = 0;
    if(color_data->r > 0) {
        if(color_data->g > 0) {
            if(color_data->b > 0) {
                color_code = 7;
            }
            else {
                color_code = 4;
            }
        }
        else if(color_data->b > 0){
            color_code = 5;
        } else {
            color_code = 1;
        }
    } else if(color_data->g > 0) {
        if(color_data->b > 0) {
            color_code = 6;
        } else {
            color_code = 2;
        }
    } else if(color_data->b > 0) {
        color_code = 3;
    }
    return color_code;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_plc_modbus");
    plc_modbus_manager mm;
    return 0;
}
