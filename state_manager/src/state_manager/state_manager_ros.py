#!/usr/bin/env python
"""
@package state_manager
@file state_manager_ros.py
@author Norawit Nangsue
@brief State Manager

Copyright (C) FIBO
FIBO
"""

from copy import deepcopy
import rospy

# ROS message & services includes
from actionlib_msgs.msg import GoalStatusArray

# other includes
from state_manager import state_manager_impl


class StateManagerROS(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_data_ = state_manager_impl.StateManagerData()
        self.component_config_ = state_manager_impl.StateManagerConfig()
        self.component_implementation_ = state_manager_impl.StateManagerImplementation()

        # handling subscribers
        self.move_base_status_ = rospy.Subscriber('move_base_status', GoalStatusArray, self.topic_callback_move_base_status)

    def topic_callback_move_base_status(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_move_base_status = msg
        self.component_data_.in_move_base_status_updated = True

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
    rospy.init_node("state_manager", anonymous=False)

    node = StateManagerROS()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.Timer(rospy.Duration(1.0 / 10), node.update)
    rospy.spin()
    node.component_implementation_.terminate()
