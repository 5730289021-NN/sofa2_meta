#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <cmath>

namespace dynamixel_tcp
{
    class DynamixelTwistJoy
    {
    public:
        DynamixelTwistJoy() : joy_updated(false)
        {
            ros::NodeHandle nodeHandle("~");
            ros::Publisher target_publisher = nodeHandle.advertise<sensor_msgs::JointState>("/head/target_position", 5);
            ros::Subscriber target_subscriber = nodeHandle.subscribe("/joy", 3, &DynamixelTwistJoy::latchJoyCallback, this);

            ros::Subscriber current_subscriber = nodeHandle.subscribe("/head/current_position", 1, &DynamixelTwistJoy::latchJointStateCallback, this);
            ros::Rate loop_rate(15);
            target_state = getZeroVelocityMessage();

            while (ros::ok())
            {
                if (joy_updated)
                {
                    joy_updated = false;
                    if (joy_msg.axes.size() == 8 && joy_msg.buttons.size() == 11)
                    {
                        /*
                        Joy Buttons 6 : Enable Head/Camera Control
                        */
                        if (joy_msg.buttons[6])
                        {
                            /*
                            Calculate Value for Joint 0
                            Joy Axes 2: Camera Up
                            Joy Axes 5: Camera Down
                            */
                            target_state.velocity[0] = abs(joy_msg.axes[2] - joy_msg.axes[5]) * 100;
                            if (joy_msg.axes[2] - joy_msg.axes[5] > 0)
                            {
                                target_state.position[0] = 600;
                            }
                            else
                            {
                                target_state.position[0] = 1400;
                            }
                            /*
                            Calculate Value for Joint 1   
                            Joy Axes 3: Robot Head-No Axis
                            */
                            target_state.velocity[1] = abs(joy_msg.axes[3]) * 100;
                            if (joy_msg.axes[3] > 0)
                            {
                                target_state.position[1] = 2700;
                            }
                            else
                            {
                                target_state.position[1] = 1400;
                            }
                            /*
                            Calculate Value for Joint 2
                            Joy Axes 4: Robot Head-Yes Axis
                            */
                            target_state.velocity[2] = abs(joy_msg.axes[4]) * 30;
                            if (joy_msg.axes[4] > 0)
                            {
                                target_state.position[2] = 1950;
                            }
                            else
                            {
                                target_state.position[2] = 2020;
                            }
                            target_publisher.publish(target_state);
                        }
                        else
                        {
                            sensor_msgs::JointState zero_vel_msg;
                            zero_vel_msg = getZeroVelocityMessage();
                            target_publisher.publish(zero_vel_msg);
                        }
                    }
                    else
                    {
                        ROS_ERROR("Joy Incompatible Error");
                    }
                }
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        ~DynamixelTwistJoy()
        {
        }

    private:
        void latchJoyCallback(const sensor_msgs::Joy &joy_in_msg)
        {
            /*Latch Desire Position*/
            joy_updated = true;
            joy_msg = joy_in_msg;
        }

        void latchJointStateCallback(const sensor_msgs::JointState &joint_in_msg)
        {
            /*Latch Current Position*/
            current_state = joint_in_msg;
        }

        sensor_msgs::JointState getZeroVelocityMessage()
        {
            sensor_msgs::JointState zero_vel_msg;
            zero_vel_msg.name.push_back("0");
            zero_vel_msg.name.push_back("1");
            zero_vel_msg.name.push_back("2");

            zero_vel_msg.position.push_back(1000);
            zero_vel_msg.position.push_back(2050);
            zero_vel_msg.position.push_back(2000);

            zero_vel_msg.velocity.push_back(0);
            zero_vel_msg.velocity.push_back(0);
            zero_vel_msg.velocity.push_back(0);
            return zero_vel_msg;
        }

        sensor_msgs::Joy joy_msg;
        sensor_msgs::JointState target_state;
        sensor_msgs::JointState current_state;
        bool joy_updated;
    };
} // namespace dynamixel_tcp

int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::init(argc, argv, "dynamixel_twist_joy");
    dynamixel_tcp::DynamixelTwistJoy dtj;
    return 0;
}
