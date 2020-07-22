#!/usr/bin/env python
"""
@package costmap_based_vel_filter
@file costmap_based_vel_filter_impl.py
@author Norawit Nangsue
@brief Velocity Filter based on provided costmap

Copyright (C) FIBO
FIBO
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Twist

# protected region user include package begin #
from math import pi, cos, sin
from geometry_msgs.msg import Pose
from itertools import chain
# protected region user include package end #


class CostmapBasedVelFilterConfig(object):
    """
    set of static and dynamic parameters
    autogenerated: don't touch this class
    """
    def __init__(self):
        # parameters handled through the parameter server
        self.radius = 0.5
        self.forecasting_time = 0.5
        self.h = 5
        self.polyno = 5
        self.max_cost_threshold = 85
        self.Kc = 1.5
        pass

    def __str__(self):
        msg = "Instance of CostmapBasedVelFilterConfig class: {"
        msg += "radius: {} ".format(self.radius)
        msg += "forecasting_time: {} ".format(self.forecasting_time)
        msg += "h: {} ".format(self.h)
        msg += "polyno: {} ".format(self.polyno)
        msg += "max_cost_threshold: {} ".format(self.max_cost_threshold)
        msg += "Kc: {} ".format(self.Kc)
        msg += "}"
        return msg


class CostmapBasedVelFilterData(object):
    """
    set of input / output handled through the update methods
    autogenerated: don't touch this class
    """
    def __init__(self):
        """
        Definition of the CostmapBasedVelFilterData attributes
        """
        # input data
        self.in_amcl_pose = PoseWithCovarianceStamped()
        self.in_amcl_pose_updated = bool()
        self.in_costmap = OccupancyGrid()
        self.in_costmap_updated = bool()
        self.in_control_mode = String()
        self.in_control_mode_updated = bool()
        pass

    def __str__(self):
        msg = "Instance of CostmapBasedVelFilterData class: \n {"
        msg += "in_amcl_pose: {} \n".format(self.in_amcl_pose)
        msg += "in_amcl_pose_updated: {} \n".format(self.in_amcl_pose_updated)
        msg += "in_costmap: {} \n".format(self.in_costmap)
        msg += "in_costmap_updated: {} \n".format(self.in_costmap_updated)
        msg += "in_control_mode: {} \n".format(self.in_control_mode)
        msg += "in_control_mode_updated: {} \n".format(self.in_control_mode_updated)
        msg += "}"
        return msg


class CostmapBasedVelFilterPassthrough(object):
    """
    set of passthrough elements slightly violating interface / implementation separation
    Autogenerated: don't touch this class
    """
    def __init__(self):
        """ Class to contain variable breaking the interface separation
        """
        self.pub_pose_array = None
        self.pub_vel_out = None
        self.sub_vel_in = None
        pass


class CostmapBasedVelFilterImplementation(object):
    """
    Class to contain Developer implementation.
    """
    def __init__(self):
        """
        Definition and initialisation of class attributes
        """
        self.passthrough = CostmapBasedVelFilterPassthrough()

        # protected region user member variables begin #
        self.pose_lists = []
        self.max_feasable_cost = 100
        # protected region user member variables end #

    def configure(self, config):
        """
        @brief configuration of the implementation
        @param      self The object
        @param      config set of configuration parameters
        @return True on success
        """
        # protected region user configure begin #
        self.enb = True
        self.config = config
        self.time_resolution = config.forecasting_time / config.h
        rospy.loginfo('Costmap based Velocity Control Started')
        return True
        # protected region user configure end #

    def update(self, data, config):
        """
        @brief { function_description }

        @param      self The object
        @param      data data handled through the ros class
        @param      config parameters handled through dyn. recon.

        @return nothing
        """
        # protected region user update begin #
        self.enb = not (data.in_control_mode.data == 'FORCE' or data.in_control_mode.data == 'NAVIGATION' or data.in_control_mode.data == 'FORCE_OVERRIDE')
        self.current_pose = data.in_amcl_pose.pose.pose
        self.costmap = data.in_costmap
        pass
        # protected region user update end #

    def terminate(self):
        """
        A function performed when Keyboard Interrupt trigger
	    This gives you a chance to save important data or clean clean object if needed
        """
        # protected region user terminate begin #
        pass
        # protected region user terminate end #


    def direct_topic_callback_vel_in(self, msg):
        """
        Direct callback at reception of message on topic vel_in
        """
        # protected region user implementation of direct subscriber callback for vel_in begin #
        if self.enb:
            for h in range(self.config.h):
                vertice_h = []
                pose_h = Pose()
                pose_h.orientation.z = self.current_pose.orientation.z + msg.angular.z * self.time_resolution * h
                pose_h.position.x = self.current_pose.position.x + msg.linear.x * self.time_resolution * h * cos(pose_h.orientation.z)
                pose_h.position.y = self.current_pose.position.y + msg.linear.x * self.time_resolution * h * sin(pose_h.orientation.z)
                for v in range(self.config.polyno):
                    vertex_pose = Pose()
                    vertex_orientation = pose_h.orientation.z + v / self.config.polyno * 2 * pi
                    vertex_pose.position.x = pose_h.position.x + h + self.config.radius * cos(vertex_orientation)
                    vertex_pose.position.y = pose_h.position.y + self.config.radius * sin(vertex_orientation)
                    vertex_pose.orientation = pose_h.orientation
                    if self.checkPoseValidity(vertex_pose):
                        vertice_h.append(vertex_pose)
                self.pose_lists.append(vertice_h)
            unnested_poses = list(chain.from_iterable(self.pose_lists))
            pose_array = PoseArray()
            pose_array.poses = unnested_poses
            self.passthrough.pub_pose_array.publish(pose_array)

            max_current_cost = 0
            for pose in unnested_poses:
                current_cost = self.getCostfromPose(pose) 
                if max_current_cost < current_cost:
                    max_current_cost = current_cost
            if max_current_cost > self.max_feasable_cost:
                rospy.logerr('Unexpected cost %d but assigned to %d' % (max_current_cost, self.max_feasable_cost))
                max_current_cost = self.max_feasable_cost

            max_cost_mul = max_current_cost * self.config.Kc 

            vel_out = Twist()
            if max_current_cost > self.config.max_cost_threshold:
                rospy.loginfo('Cost beyond than threshold %d, but current cost is %f' % (self.config.max_cost_threshold, max_cost_mul))
            else:
                vel_out.linear.x = msg.linear.x * (1 - max_cost_mul / self.max_feasable_cost)
                vel_out.angular.z = msg.angular.z
            self.passthrough.pub_vel_out.publish(vel_out)
        else:
            self.passthrough.pub_vel_out.publish(msg)
        # protected region user implementation of direct subscriber callback for vel_in end #
        pass
    # protected region user additional functions begin #
    def checkPoseValidity(self, pose):
        height_cond = (pose.position.y - self.costmap.info.origin.position.y) / self.costmap.info.resolution < self.costmap.info.height
        width_cond = (pose.position.x - self.costmap.info.origin.position.x) / self.costmap.info.resolution < self.costmap.info.width
        pose_valid = height_cond and width_cond
        if not pose_valid:
            rospy.loginfo('Invalid Pose at %f, %f' % (pose.position.x, pose.position.y))
        return pose_valid

    def getCostfromPose(self, pose):
        index = (pose.position.y - self.costmap.info.origin.position.y) / self.costmap.info.resolution * self.costmap.info.width
        index += (pose.position.x - self.costmap.info.origin.position.x) / self.costmap.info.resolution
        return self.costmap.data[index]
    # protected region user additional functions end #
