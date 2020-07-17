#!/usr/bin/env python
"""
@package costmap_based_vel_filter
@file costmap_based_vel_filter_ros.py
@author Norawit Nangsue
@brief Velocity Filter based on provided costmap

Copyright (C) FIBO
FIBO
"""

from copy import deepcopy
import rospy

# ROS message & services includes
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist

# other includes
from costmap_based_vel_filter import costmap_based_vel_filter_impl


class CostmapBasedVelFilterROS(object):
    """
    ROS interface class, handling all communication with ROS
    """
    def __init__(self):
        """
        Attributes definition
        """
        self.component_data_ = costmap_based_vel_filter_impl.CostmapBasedVelFilterData()
        self.component_config_ = costmap_based_vel_filter_impl.CostmapBasedVelFilterConfig()
        self.component_implementation_ = costmap_based_vel_filter_impl.CostmapBasedVelFilterImplementation()

        # handling parameters from the parameter server
        self.component_config_.radius = rospy.get_param("~radius", 0.5)
        self.component_config_.forecasting_time = rospy.get_param("~forecasting_time", 0.5)
        self.component_config_.h = rospy.get_param("~h", 5)
        self.component_config_.polyno = rospy.get_param("~polyno", 5)
        self.component_config_.max_cost_threshold = rospy.get_param("~max_cost_threshold", 85)
        self.component_config_.Kc = rospy.get_param("~Kc", 1.5)
        # handling subscribers
        self.amcl_pose_ = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.topic_callback_amcl_pose)
        self.costmap_ = rospy.Subscriber('costmap', OccupancyGrid, self.topic_callback_costmap)
        self.control_mode_ = rospy.Subscriber('control_mode', String, self.topic_callback_control_mode)
        # Handling direct publisher
        self.component_implementation_.passthrough.pub_pose_array = rospy.Publisher('pose_array', PoseArray, queue_size=1)
        self.component_implementation_.passthrough.pub_vel_out = rospy.Publisher('vel_out', Twist, queue_size=1)
        # Handling direct subscriber
        self.component_implementation_.passthrough.sub_vel_in = rospy.Subscriber('vel_in',
                                                                                 Twist,
                                                                                 self.component_implementation_.direct_topic_callback_vel_in)

    def topic_callback_amcl_pose(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_amcl_pose = msg
        self.component_data_.in_amcl_pose_updated = True

    def topic_callback_costmap(self, msg):
        """
        callback called at message reception
        """
        self.component_data_.in_costmap = msg
        self.component_data_.in_costmap_updated = True

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
        pass

    def set_all_input_read(self):
        """
        set related flag to state that input has been read
        """
        self.component_data_.in_amcl_pose_updated = False
        self.component_data_.in_costmap_updated = False
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


def main():
    """
    @brief Entry point of the package.
    Instanciate the node interface containing the Developer implementation
    @return nothing
    """
    rospy.init_node("costmap_based_vel_filter", anonymous=False)

    node = CostmapBasedVelFilterROS()
    if not node.configure():
        rospy.logfatal("Could not configure the node")
        rospy.logfatal("Please check configuration parameters")
        rospy.logfatal("{}".format(node.component_config_))
        return

    rospy.Timer(rospy.Duration(1.0 / 10), node.update)
    rospy.spin()
    node.component_implementation_.terminate()
