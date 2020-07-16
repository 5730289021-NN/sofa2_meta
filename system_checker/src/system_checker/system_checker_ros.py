#!/usr/bin/env python
"""
@package system_checker
@file system_checker_ros.py
@author Norawit Nangsue
@brief System Checker

Copyright (C) FIBO
FIBO
"""

from copy import deepcopy
import rospy

# ROS message & services includes
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import LaserScan

# other includes
from system_checker import system_checker_impl


class SystemCheckerROS(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_data_ = system_checker_impl.SystemCheckerData()
        self.component_config_ = system_checker_impl.SystemCheckerConfig()
        self.component_implementation_ = system_checker_impl.SystemCheckerImplementation()

        # handling subscribers
        self.move_base_status_ = rospy.Subscriber('move_base_status', GoalStatusArray, self.topic_callback_move_base_status)
        self.scan_1_ = rospy.Subscriber('scan_1', LaserScan, self.topic_callback_scan_1)
        self.scan_2_ = rospy.Subscriber('scan_2', LaserScan, self.topic_callback_scan_2)
        self.scan_fusion_ = rospy.Subscriber('scan_fusion', LaserScan, self.topic_callback_scan_fusion)

    def topic_callback_move_base_status(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_move_base_status = msg
        self.component_data_.in_move_base_status_updated = True

    def topic_callback_scan_1(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_scan_1 = msg
        self.component_data_.in_scan_1_updated = True

    def topic_callback_scan_2(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_scan_2 = msg
        self.component_data_.in_scan_2_updated = True

    def topic_callback_scan_fusion(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_scan_fusion = msg
        self.component_data_.in_scan_fusion_updated = True

    def configure(self):
        """
        function setting the initial configuration of the node
        """
        return self.component_implementation_.configure(self.component_config_)

    def activate_all_output(self):
        """
        activate all defined output
        """
        pass

    def set_all_input_read(self):
        """
        set related flag to state that input has been read
        """
        self.component_data_.in_move_base_status_updated = False
        self.component_data_.in_scan_1_updated = False
        self.component_data_.in_scan_2_updated = False
        self.component_data_.in_scan_fusion_updated = False
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


def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("system_checker", anonymous=False)

    node = SystemCheckerROS()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.Timer(rospy.Duration(1.0 / 1), node.update)
    rospy.spin()
    node.component_implementation_.terminate()
