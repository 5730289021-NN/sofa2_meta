#!/usr/bin/env python
"""
@package roboteq_tcp_driver
@file driver_ros.py
@author Norawit Nangsue
@brief ROS Roboteq Driver

Copyright (C) FIBO
FIBO
"""

from copy import deepcopy
import rospy

# ROS message & services includes
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# other includes
from roboteq_tcp_driver import driver_impl


class DriverROS(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_data_ = driver_impl.DriverData()
        self.component_config_ = driver_impl.DriverConfig()
        self.component_implementation_ = driver_impl.DriverImplementation()

        # handling parameters from the parameter server
        self.component_config_.ip_addr = rospy.get_param("~ip_addr", "192.168.200.171")
        self.component_config_.port_num = rospy.get_param("~port_num", 9001)
        self.component_config_.wheel_circ = rospy.get_param("~wheel_circ", 0.47124)
        self.component_config_.track_width = rospy.get_param("~track_width", 0.34)
        self.component_config_.cpr = rospy.get_param("~cpr", 200000)
        self.component_config_.gear_ratio = rospy.get_param("~gear_ratio", 20)
        self.component_config_.invert_mul = rospy.get_param("~invert_mul", -1)
        # handling publishers
        self.odom_ = rospy.Publisher('odom', Odometry, queue_size=1)
        self.queryResult_ = rospy.Publisher('queryResult', String, queue_size=1)
        # handling subscribers
        self.queryCommand_ = rospy.Subscriber('queryCommand', String, self.topic_callback_queryCommand)
        self.cmd_vel_ = rospy.Subscriber('cmd_vel', Twist, self.topic_callback_cmd_vel)

    def topic_callback_queryCommand(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_queryCommand = msg
        self.component_data_.in_queryCommand_updated = True

    def topic_callback_cmd_vel(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_cmd_vel = msg
        self.component_data_.in_cmd_vel_updated = True

    def configure(self):
        """
        function setting the initial configuration of the node
        """
        return self.component_implementation_.configure(self.component_config_)

    def activate_all_output(self):
        """
        activate all defined output
        """
        self.component_data_.out_odom_active = True
        self.component_data_.out_queryResult_active = True
        pass

    def set_all_input_read(self):
        """
        set related flag to state that input has been read
        """
        self.component_data_.in_queryCommand_updated = False
        self.component_data_.in_cmd_vel_updated = False
        pass

    def update(self, event):
        """
        @brief update function

        @param      self The object
        @param      event The event

        @return { description_of_the_return_value }
        """
        self.activate_all_output()
        config = deepcopy(self.component_config_)
        data = deepcopy(self.component_data_)
        self.set_all_input_read()
        self.component_implementation_.update(data, config)

        try:
            self.component_data_.out_odom_active = data.out_odom_active
            self.component_data_.out_odom = data.out_odom
            if self.component_data_.out_odom_active:
                self.odom_.publish(self.component_data_.out_odom)
            self.component_data_.out_queryResult_active = data.out_queryResult_active
            self.component_data_.out_queryResult = data.out_queryResult
            if self.component_data_.out_queryResult_active:
                self.queryResult_.publish(self.component_data_.out_queryResult)
        except rospy.ROSException as error:
            rospy.logerr("Exception: {}".format(error))


def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("driver", anonymous=False, log_level=rospy.DEBUG)

    node = DriverROS()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.Timer(rospy.Duration(1.0 / 5), node.update)
    rospy.spin()
    node.component_implementation_.terminate()
