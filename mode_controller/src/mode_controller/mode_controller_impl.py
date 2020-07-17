#!/usr/bin/env python
"""
@package mode_controller
@file mode_controller_impl.py
@author Norawit Nangsue
@brief Custom Mode Controller for SOFA Robot

Copyright (C) FIBO
FIBO
"""

import rospy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Bool

# protected region user include package begin #
# protected region user include package end #


class ModeControllerConfig(object):
    """
    set of static and dynamic parameters
    autogenerated: don't touch this class
    """
    def __init__(self):
        # parameters handled through the parameter server
        self.manual_btn = 0
        self.cancel_btn = 3
        self.force_btn = 1
        pass

    def __str__(self):
        msg = "Instance of ModeControllerConfig class: {"
        msg += "manual_btn: {} ".format(self.manual_btn)
        msg += "cancel_btn: {} ".format(self.cancel_btn)
        msg += "force_btn: {} ".format(self.force_btn)
        msg += "}"
        return msg


class ModeControllerData(object):
    """
    set of input / output handled through the update methods
    autogenerated: don't touch this class
    """
    def __init__(self):
        """
        Definition of the ModeControllerData attributes
        """
        # input data
        self.in_move_base_status = GoalStatusArray()
        self.in_move_base_status_updated = bool()
        self.in_joy = Joy()
        self.in_joy_updated = bool()
        self.in_follow_me_status = String()
        self.in_follow_me_status_updated = bool()
        # output data
        self.out_control_mode = String()
        self.out_control_mode_active = bool()
        pass

    def __str__(self):
        msg = "Instance of ModeControllerData class: \n {"
        msg += "in_move_base_status: {} \n".format(self.in_move_base_status)
        msg += "in_move_base_status_updated: {} \n".format(self.in_move_base_status_updated)
        msg += "in_joy: {} \n".format(self.in_joy)
        msg += "in_joy_updated: {} \n".format(self.in_joy_updated)
        msg += "in_follow_me_status: {} \n".format(self.in_follow_me_status)
        msg += "in_follow_me_status_updated: {} \n".format(self.in_follow_me_status_updated)
        msg += "out_control_mode: {} \n".format(self.out_control_mode_active)
        msg += "out_control_mode_active: {} \n".format(self.out_control_mode_active)
        msg += "}"
        return msg


class ModeControllerPassthrough(object):
    """
    set of passthrough elements slightly violating interface / implementation separation
    Autogenerated: don't touch this class
    """
    def __init__(self):
        """ Class to contain variable breaking the interface separation
        """
        self.pub_move_base_cancel = None
        self.pub_follow_me_enable = None
        pass


class ModeControllerImplementation(object):
    """
    Class to contain Developer implementation.
    """
    def __init__(self):
        """
        Definition and initialisation of class attributes
        """
        self.passthrough = ModeControllerPassthrough()

        # protected region user member variables begin #
        self.force_mode = False
        self.manual_mode = False
        self.follow_me_mode = False
        self.navigation_mode = False
        
        self.autonomous_mode = False
        # protected region user member variables end #

    def configure(self, config):
        """
        @brief configuration of the implementation
        @param      self The object
        @param      config set of configuration parameters
        @return True on success
        """
        # protected region user configure begin #
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
        self.force_mode = bool(data.in_joy.buttons[config.force_btn])
        self.manual_mode = bool(data.in_joy.buttons[config.manual_btn])
        self.follow_me_mode = rospy.get_param(config.follow_me_status_param, 'UNFOLLOWED') == 'FOLLOWING'
        self.navigation_mode = data.in_move_base_status.status_list and data.in_move_base_status.status_list[-1].status == 1
        self.autonomous_mode = self.follow_me_mode or self.navigation_mode

        if bool(data.in_joy.buttons[config.cancel_btn]):
            rospy.loginfo('Cancel Button Triggered')
            if self.navigation_mode:
                rospy.loginfo('Navigation Canceled')
                self.passthrough.pub_move_base_cancel.publish(GoalID())
            elif self.follow_me_mode:
                rospy.loginfo('Follow Me Canceled')
                self.passthrough.pub_follow_me_enable.publish(Bool())
        
        # modes = ['STAND_BY', 'MANUAL', 'OVERRIDE', 'FORCE', 'FORCE_OVERRIDE', 'FOLLOW_ME', 'NAVIGATION', 'UNKNOWN']
        data.out_control_mode.data = self.getCurrentMode()
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


    # protected region user additional functions begin #
    def getCurrentMode(self):
        if self.force_mode and self.autonomous_mode:
            return 'FORCE_OVERRIDE'
        elif self.force_mode:
            return 'FORCE'

        if self.manual_mode and self.autonomous_mode:
            return 'MANUAL_OVERRIDE'
        elif self.manual_mode:
            return 'MANUAL'
        
        if not self.autonomous_mode:
            return 'STAND_BY'
        elif self.follow_me_mode:
            return 'FOLLOW_ME'
        elif self.navigation_mode:
            return 'NAVIGATION'
        
        rospy.logerr('Unknown Mode arrived')
        return 'UNKNOWN'

    # protected region user additional functions end #
