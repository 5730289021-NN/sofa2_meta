#!/usr/bin/env python
"""
@package mode_controller
@file mode_controller_ros.py
@author Norawit Nangsue
@brief Custom Mode Controller for SOFA Robot

Copyright (C) FIBO
FIBO
"""

from copy import deepcopy
import rospy

# ROS message & services includes
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist

# other includes
from mode_controller import mode_controller_impl


class ModeControllerROS(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_data_ = mode_controller_impl.ModeControllerData()
        self.component_config_ = mode_controller_impl.ModeControllerConfig()
        self.component_implementation_ = mode_controller_impl.ModeControllerImplementation()

        # handling parameters from the parameter server
        self.component_config_.follow_me_status_param = rospy.get_param("~follow_me_status_param", "/follow_me_status_param")
        self.component_config_.safe_btn = rospy.get_param("~safe_btn", 0)
        self.component_config_.cancel_btn = rospy.get_param("~cancel_btn", 3)
        self.component_config_.raw_btn = rospy.get_param("~raw_btn", 1)
        # handling publishers
        self.control_mode_ = rospy.Publisher('control_mode', String, queue_size=1)
        # handling subscribers
        self.move_base_status_ = rospy.Subscriber('move_base_status', GoalStatusArray, self.topic_callback_move_base_status)
        self.joy_ = rospy.Subscriber('joy', Joy, self.topic_callback_joy)
        # Handling direct publisher
        self.component_implementation_.passthrough.pub_move_base_cancel = rospy.Publisher('move_base_cancel', GoalID, queue_size=1)
        self.component_implementation_.passthrough.pub_follow_me_enable = rospy.Publisher('follow_me_enable', Bool, queue_size=1)
        self.component_implementation_.passthrough.pub_joy_final_vel = rospy.Publisher('joy_final_vel', Twist, queue_size=1)
        # Handling direct subscriber
        self.component_implementation_.passthrough.sub_joy_raw_vel = rospy.Subscriber('joy_raw_vel',
                                                                                 Twist,
                                                                                 self.component_implementation_.direct_topic_callback_joy_raw_vel)
        self.component_implementation_.passthrough.sub_joy_safe_vel = rospy.Subscriber('joy_safe_vel',
                                                                                 Twist,
                                                                                 self.component_implementation_.direct_topic_callback_joy_safe_vel)

    def topic_callback_move_base_status(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_move_base_status = msg
        self.component_data_.in_move_base_status_updated = True

    def topic_callback_joy(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_joy = msg
        self.component_data_.in_joy_updated = True

    def configure(self):
        """
        function setting the initial configuration of the node
        """
        return self.component_implementation_.configure(self.component_config_)

    def activate_all_output(self):
        """
        activate all defined output
        """
        self.component_data_.out_control_mode_active = True
        pass

    def set_all_input_read(self):
        """
        set related flag to state that input has been read
        """
        self.component_data_.in_move_base_status_updated = False
        self.component_data_.in_joy_updated = False
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
            self.component_data_.out_control_mode_active = data.out_control_mode_active
            self.component_data_.out_control_mode = data.out_control_mode
            if self.component_data_.out_control_mode_active:
                self.control_mode_.publish(self.component_data_.out_control_mode)
        except rospy.ROSException as error:
            rospy.logerr("Exception: {}".format(error))


def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("mode_controller", anonymous=False)

    node = ModeControllerROS()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.Timer(rospy.Duration(1.0 / 10), node.update)
    rospy.spin()
    node.component_implementation_.terminate()
