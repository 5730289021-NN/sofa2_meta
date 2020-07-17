#!/usr/bin/env python
"""
@package acc_lim_vel_filter
@file acc_lim_vel_filter_ros.py
@author Norawit Nangsue
@brief Linear Acceleration Limit Velocity Filter

Copyright (C) FIBO
FIBO
"""

from copy import deepcopy
import rospy

# ROS message & services includes
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String

# other includes
from acc_lim_vel_filter import acc_lim_vel_filter_impl


class AccLimVelFilterROS(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_data_ = acc_lim_vel_filter_impl.AccLimVelFilterData()
        self.component_config_ = acc_lim_vel_filter_impl.AccLimVelFilterConfig()
        self.component_implementation_ = acc_lim_vel_filter_impl.AccLimVelFilterImplementation()

        # handling parameters from the parameter server
        self.component_config_.acc_lin_lim = rospy.get_param("~acc_lin_lim", 0.6)
        self.component_config_.goal_dist_thres = rospy.get_param("~goal_dist_thres", 1.5)
        self.component_config_.auto_min_vel = rospy.get_param("~auto_min_vel", 0.1)
        self.component_config_.Kp = rospy.get_param("~Kp", 0.1)
        # handling publishers
        self.vel_out_ = rospy.Publisher('vel_out', Twist, queue_size=1)
        # handling subscribers
        self.vel_in_ = rospy.Subscriber('vel_in', Twist, self.topic_callback_vel_in)
        self.amcl_pose_ = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.topic_callback_amcl_pose)
        self.move_base_goal_ = rospy.Subscriber('move_base_goal', MoveBaseActionGoal, self.topic_callback_move_base_goal)
        self.control_mode_ = rospy.Subscriber('control_mode', String, self.topic_callback_control_mode)

    def topic_callback_vel_in(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_vel_in = msg
        self.component_data_.in_vel_in_updated = True

    def topic_callback_amcl_pose(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_amcl_pose = msg
        self.component_data_.in_amcl_pose_updated = True

    def topic_callback_move_base_goal(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_move_base_goal = msg
        self.component_data_.in_move_base_goal_updated = True

    def topic_callback_control_mode(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_control_mode = msg
        self.component_data_.in_control_mode_updated = True

    def configure(self):
        """
        function setting the initial configuration of the node
        """
        return self.component_implementation_.configure(self.component_config_)

    def activate_all_output(self):
        """
        activate all defined output
        """
        self.component_data_.out_vel_out_active = True
        pass

    def set_all_input_read(self):
        """
        set related flag to state that input has been read
        """
        self.component_data_.in_vel_in_updated = False
        self.component_data_.in_amcl_pose_updated = False
        self.component_data_.in_move_base_goal_updated = False
        self.component_data_.in_control_mode_updated = False
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
            self.component_data_.out_vel_out_active = data.out_vel_out_active
            self.component_data_.out_vel_out = data.out_vel_out
            if self.component_data_.out_vel_out_active:
                self.vel_out_.publish(self.component_data_.out_vel_out)
        except rospy.ROSException as error:
            rospy.logerr("Exception: {}".format(error))


def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("acc_lim_vel_filter", anonymous=False)

    node = AccLimVelFilterROS()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.Timer(rospy.Duration(1.0 / 20), node.update)
    rospy.spin()
    node.component_implementation_.terminate()
