#!/usr/bin/env python
"""
@package roboteq_tcp_driver
@file driver_impl.py
@author Norawit Nangsue
@brief ROS Roboteq Driver

Copyright (C) FIBO
FIBO
"""

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tf

# protected region user include package begin #
from geometry_msgs.msg import Quaternion
from math import sin, cos
import socket
import time
# protected region user include package end #


class DriverConfig(object):
    """
    set of static and dynamic parameters
    autogenerated: don't touch this class
    """
    def __init__(self):
        # parameters handled through the parameter server
        self.ip_addr = "192.168.200.171"
        self.port_num = 9001
        self.wheel_circ = 0.47124
        self.track_width = 0.34
        self.cpr = 200000
        self.gear_ratio = 20
        self.invert_mul = 1
        pass

    def __str__(self):
        msg = "Instance of DriverConfig class: {"
        msg += "ip_addr: {} ".format(self.ip_addr)
        msg += "port_num: {} ".format(self.port_num)
        msg += "wheel_circ: {} ".format(self.wheel_circ)
        msg += "track_width: {} ".format(self.track_width)
        msg += "cpr: {} ".format(self.cpr)
        msg += "gear_ratio: {} ".format(self.gear_ratio)
        msg += "invert_mul: {} ".format(self.invert_mul)
        msg += "}"
        return msg


class DriverData(object):
    """
    set of input / output handled through the update methods
    autogenerated: don't touch this class
    """
    def __init__(self):
        """
        Definition of the DriverData attributes
        """
        # input data
        self.in_queryCommand = String()
        self.in_queryCommand_updated = bool()
        self.in_cmd_vel = Twist()
        self.in_cmd_vel_updated = bool()
        # output data
        self.out_odom = Odometry()
        self.out_odom_active = bool()
        self.out_queryResult = String()
        self.out_queryResult_active = bool()
        pass

    def __str__(self):
        msg = "Instance of DriverData class: \n {"
        msg += "in_queryCommand: {} \n".format(self.in_queryCommand)
        msg += "in_queryCommand_updated: {} \n".format(self.in_queryCommand_updated)
        msg += "in_cmd_vel: {} \n".format(self.in_cmd_vel)
        msg += "in_cmd_vel_updated: {} \n".format(self.in_cmd_vel_updated)
        msg += "out_odom: {} \n".format(self.out_odom_active)
        msg += "out_odom_active: {} \n".format(self.out_odom_active)
        msg += "out_queryResult: {} \n".format(self.out_queryResult_active)
        msg += "out_queryResult_active: {} \n".format(self.out_queryResult_active)
        msg += "}"
        return msg


class DriverPassthrough(object):
    """
    set of passthrough elements slightly violating interface / implementation separation
    Autogenerated: don't touch this class
    """
    def __init__(self):
        """ Class to contain variable breaking the interface separation
        """
        self.odom_to_base_footprint = tf.TransformBroadcaster()
        pass


class DriverImplementation(object):
    """
    Class to contain Developer implementation.
    """
    def __init__(self):
        """
        Definition and initialisation of class attributes
        """
        self.passthrough = DriverPassthrough()

        # protected region user member variables begin #
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.data_sock = ''
        self.socket_mutex = False
        self.error = ''
        self.transaction_stack_cnt = 0
        # protected region user member variables end #

    def configure(self, config):
        """
        @brief configuration of the implementation
        @param      self The object
        @param      config set of configuration parameters
        @return True on success
        """
        # protected region user configure begin #
        rospy.loginfo('Connecting to Roboteq via TCP Socket....')
        while True:
            try:
                self.sock.connect((config.ip_addr, config.port_num))
                break
            except socket.error as msg:
                if 'Errno 113' in str(msg):
                    self.set_error('DRIVER_CONNECTING')
                else:
                    self.set_error(msg)
                rospy.loginfo('Trying to reconnecting in 5 Seconds...')
                time.sleep(5.0)
        self.set_error('')
        self.sock.settimeout(0.1)
        # self.sock.setblocking(0)
        rospy.loginfo('Roboteq Driver successfully connected using blocking mode')
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

        # Retrieve Encoder Data
        self.socket_transceive('?CR', config, description="Encoder Query")
        if self.data_sock == '':
            return
        # Wheel Odometry(http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html)
        # Extract Data
        try:
            encin = (float(self.data_sock.split(':')[0].replace('CR=', '')), float(self.data_sock.split(':')[1]))
        except ValueError as msg:
            rospy.logerr('ValueError: %s', msg)
            return
        # Calculate Forward Kinematic
        dx_robot = config.invert_mul * (encin[0] + encin[1]) * config.wheel_circ / (2 * config.cpr)
        da = config.invert_mul * (encin[1] - encin[0]) * config.wheel_circ / (config.track_width * config.cpr)
        # Obtain time and relative time from last update
        time_cur = rospy.Time.now()
        time_rel = time_cur - data.out_odom.header.stamp
        # Define frame of reference
        data.out_odom.header.frame_id = 'odom'
        data.out_odom.child_frame_id = 'base_footprint'
        # Update Odom Pose
        data.out_odom.header.stamp = time_cur
        quat_past = (0, 0, data.out_odom.pose.pose.orientation.z, data.out_odom.pose.pose.orientation.w)
        yaw_past = tf.transformations.euler_from_quaternion(quat_past)[2]
        pos_cur = (data.out_odom.pose.pose.position.x + dx_robot * cos(yaw_past), 
                   data.out_odom.pose.pose.position.y + dx_robot * sin(yaw_past), 0)
        quat_cur = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw_past + da))
        data.out_odom.pose.pose.position.x = pos_cur[0]
        data.out_odom.pose.pose.position.y = pos_cur[1]
        data.out_odom.pose.pose.orientation = quat_cur
        # Update Odom Twist
        data.out_odom.twist.twist.linear.x = dx_robot / time_rel.to_sec()
        data.out_odom.twist.twist.angular.z = da / time_rel.to_sec()
        # tf
        self.passthrough.odom_to_base_footprint.sendTransform(pos_cur, (0, 0, quat_cur.z, quat_cur.w), time_cur, "base_footprint", "odom")
        

        ### Send cmd_vel

        # Calculate Wheel RPM
        wheel_speed = (data.in_cmd_vel.linear.x - config.track_width * data.in_cmd_vel.angular.z / 2,
                       data.in_cmd_vel.linear.x + config.track_width * data.in_cmd_vel.angular.z / 2)
        rpmout = (int(config.gear_ratio * wheel_speed[0] / config.wheel_circ * 60), int(config.gear_ratio * wheel_speed[1] / config.wheel_circ * 60)) 
        # Velocity Command (cmd_vel)
        if data.in_cmd_vel_updated:
            for i in (1,2):
                self.socket_transceive('!S %d %d\r' % (i, config.invert_mul * rpmout[i-1]), config, 8, 'Motor %s' % i)
                if self.data_sock != '+\r':
                    rospy.logerr('Unexpected Device Response when Driving Command')
        ### Direct Query

        # Handle Direct Query Command
        if data.in_queryCommand_updated:
            self.socket_transceive(data.in_queryCommand.data, config, description="Direct Query")
            data.out_queryResult = self.data_sock
            data.out_queryResult_active = True
        else:
            data.out_queryResult_active = False
        rospy.logdebug_throttle_identical(0.2, '################################################')
        # protected region user update end #

    def terminate(self):
        """
        A function performed when Keyboard Interrupt trigger
	    This gives you a chance to save important data or clean clean object if needed
        """
        # protected region user terminate begin #
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        rospy.loginfo('Roboteq TCP Driver Terminated')
        # protected region user terminate end #


    # protected region user additional functions begin #
    def carryTimer(self, timer):
        self.rospyTimer = timer

    def socket_reconnect(self, config):
        """
        Reconnect socket when SocketError arise
        """
        try:
            rospy.loginfo('Performing Reconnecting Procedure...in 3 Seconds')
            time.sleep(3.0)
            rospy.loginfo('Shut down Socket')
            self.sock.shutdown(socket.SHUT_RDWR)
            rospy.loginfo('Close Socket')
            self.sock.close()
            rospy.loginfo('Reconnecting Socket...in 2 Seconds')
            time.sleep(2.0)
            self.sock.connect((config.ip_addr, config.port_num))
            rospy.loginfo('Socket is able to get reconnected')
            self.set_error('')
            self.socket_mutex = False
            self.rospyTimer.start()
        except Exception as msg:
            rospy.logerr('Driver is completely gone: %s', msg)

    def socket_transceive(self, command, config, buff_size=64, description="Query"):
        """
        Wrapper for Transmit and Receive Response from Driver Server
        This modifies self.data_sock
        """
        self.data_sock = ''
        try:
            if not self.socket_mutex and self.error == '':
                self.socket_mutex = True
                rospy.logdebug_throttle_identical(1, '%s: %s' % (description, command))
                self.sock.sendall(command + '\r')
                while self.data_sock == '' or self.data_sock[-1] != '\r': # Loop until receiving '\r'
                    self.data_sock += self.sock.recv(64)
                    rospy.logdebug_throttle_identical(1, 'Received: %s', self.data_sock)
                rospy.logdebug_throttle_identical(1, '%s Response: %s' % (description, self.data_sock))
                self.socket_mutex = False
                self.transaction_stack_cnt = 0
                return self.data_sock
            else:
                self.transaction_stack_cnt = self.transaction_stack_cnt + 1
                raise Exception('TRANSACTION_STACKED')
        except socket.error as msg:
            rospy.logerr('Socket Error(%s): %s', description, msg)
            if 'Errno 104' in str(msg) or 'Errno 9' in str(msg) or self.transaction_stack_cnt >= 5:
                #[Errno 104] Connection reset by peer
                #[Errno 9] Bad file descriptor
                self.set_error('DRIVER_SOCKET_GONE')
                #Handling Socket Close by Reconnect
                self.rospyTimer.shutdown()
                self.socket_reconnect(config)
            else:
                self.set_error(str(msg))
            return
        except Exception as msg:
            #Unknown Error Exception...
            self.set_error(str(msg))
            rospy.logerr('Trasceiving Error: %s: %s', msg, self.data_sock)
            return
    
    def set_error(self, description):
        """
        Update Error Status
        """
        if description != '':
            rospy.loginfo('/fault/driver was set to %s', description)
        rospy.set_param('/fault/driver', description)
        self.error = description
    # protected region user additional functions end #
