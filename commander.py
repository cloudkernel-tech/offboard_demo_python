#!/usr/bin/env python3

# Commander Interface for Kerloud Autocar python API
# More information can be referred in: https://kerloud-autocar.readthedocs.io/
# All rights reserved for Cloudkernel Technologies (Shenzhen) Co., Ltd.
# Author: cloudkerneltech@gmail.com

import rospy
import time
import math
import numpy as np
# from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3Stamped
from std_msgs.msg import Float32, String, Bool
from mavros_msgs.msg import Thrust, ActuatorControl


class Commander:
    def __init__(self):
        # current position in ENU frame
        self.local_position = None  # array
        # current velocity in ENU frame
        self.local_velocity = None # array

        self.forward_driving_state = True


        # flag to indicate that the mavros core is ready to work
        self.flag_core_ready = False

        rospy.init_node("commander_node")
        rate = rospy.Rate(30)

        '''publications and subscriptions'''
        self.position_target_pub = rospy.Publisher('gi/set_pose/position', PoseStamped, queue_size=5)
        self.velocity_target_pub = rospy.Publisher('gi/set_pose/velocity', Vector3Stamped, queue_size=5)
        self.attitude_target_pub = rospy.Publisher('gi/set_pose/attitude', PoseStamped, queue_size=5)
        self.thrust_target_pub = rospy.Publisher('gi/set_thrust', Thrust, queue_size=5)
        self.actuator_control_pub = rospy.Publisher('gi/set_act_control', ActuatorControl, queue_size=5)

        self.custom_activity_pub = rospy.Publisher('gi/set_activity/type', String, queue_size=5)
        self.driving_state_pub = rospy.Publisher('gi/set_driving_state', Bool, queue_size=5)

        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_velocity_sub = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.local_velocity_callback)
        self.core_ready_sub = rospy.Subscriber("gi/core_ready", Bool, self.coreready_callback)

        print("Kerloud python API commander interface initialized!")


    # callback for local position infor
    def local_pose_callback(self, msg):
        self.local_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def local_velocity_callback(self, msg):
        self.local_velocity = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.zy]

    # callback for core ready indication (armed and offboard)
    def coreready_callback(self, msg):
        self.flag_core_ready = msg.data


    ##### Action functions #####

    # move to a target position
    def move_position(self, x, y, z=0, BODY_FLU=False):
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_FLU))

    def move_velocity(self, vx, vy, vz=0, BODY_FLU=False):
        self.velocity_target_pub.publish(self.set_velocity(vx, vy, vz, BODY_FLU))

    # set desired yaw and throttle inputs for attitude control mode
    # yaw: unit rad (expressed in ENU frame), throttle: 0~1
    def set_yaw_and_throttle(self, yaw, throttle):
        # attitude setpoint from yaw, convert yaw to NED frame due to mavros plugin setting
        # q = quaternion_from_euler(0, 0, yaw)
        yaw_ned = self.wrap2pi(math.pi/2 - yaw)

        q = self.get_quaternion_from_euler(0, 0, yaw_ned)

        pose_tmp = PoseStamped()
        pose_tmp.pose.orientation.x = q[0]
        pose_tmp.pose.orientation.y = q[1]
        pose_tmp.pose.orientation.z = q[2]
        pose_tmp.pose.orientation.w = q[3]
        self.attitude_target_pub.publish(pose_tmp)

        # thrust topic
        thrust_tmp = Thrust()
        thrust_tmp.thrust = throttle
        self.thrust_target_pub.publish(thrust_tmp)

    # set servo and throttle inputs for actuator control mode
    def set_steering_servo_and_throttle(self, steering_servo, throttle):
        act_control_tmp = ActuatorControl()
        act_control_tmp.flag_rover_mode = 1
        act_control_tmp.group_mix = 0
        act_control_tmp.controls[2] = steering_servo
        act_control_tmp.controls[3] = throttle
        self.actuator_control_pub.publish(act_control_tmp)

    # set forward driving state
    def set_forward_driving(self):
        self.forward_driving_state = True
        tmp = Bool()
        tmp.data = True
        self.driving_state_pub.publish(tmp)

    # set backward driving state
    def set_backward_driving(self):
        self.forward_driving_state = False
        tmp = Bool()
        tmp.data = False
        self.driving_state_pub.publish(tmp)

    # arm the vehicle on land
    def arm(self):
        self.custom_activity_pub.publish(String("ARM"))

    # disarm the vehicle on land
    def disarm(self):
        self.custom_activity_pub.publish(String("DISARM"))

    # return to home position (ENU origin) with defined height
    def return_home(self):
        self.position_target_pub.publish(self.set_pose(0, 0, 0, False))



    ##### Supporting functions #####
    # set local position setpoint in position command mode
    def set_pose(self, x=0, y=0, z=0, BODY_FLU = False):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention
        if BODY_FLU:
            pose.header.frame_id = 'base_link'
        else:
            pose.header.frame_id = 'map' # ENU frame

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        return pose

    # set local velocity setpoint in velocity command mode
    def set_velocity(self, vx, vy, vz=0, BODY_FLU=False):
        vel = Vector3Stamped()
        vel.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention
        if BODY_FLU:
            vel.header.frame_id = 'base_link'
        else:
            vel.header.frame_id = 'map' # ENU frame

        vel.vector.x = vx
        vel.vector.y = vy
        vel.vector.z = vz

        return vel

    """limit an angle to the range [-Pi, Pi]"""
    def wrap2pi(self, angle_rad):
        if angle_rad > math.pi:
            angle_rad = angle_rad - 2.0*math.pi
        elif angle_rad < -math.pi:
            angle_rad = angle_rad + 2.0*math.pi
        return angle_rad


    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        ref: aircraft control and simulation
        Input
          :param roll: The roll (rotation around x-axis) angle in radians.
          :param pitch: The pitch (rotation around y-axis) angle in radians.
          :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
          :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)

        return [qx, qy, qz, qw]

if __name__ == "__main__":

    # initialize the commanding interface
    con = Commander()
    time.sleep(2)

    # waiting for core to be ready
    while not con.flag_core_ready:
        print("waiting for the vehicle to be armed and in offboard")
        time.sleep(1)

    # rover mission
    # position guidance cmd
    print("Commander interface: start position control mode")
    con.move_position(5, 0, 0)
    print("Rover: go to position (5,0,0)")
    time.sleep(5)
    con.move_position(5, 5, 0)
    print("Rover: go to position (5,5,0)")
    time.sleep(5)

    # velocity guidance cmd
    print("Commander interface: start velocity control mode")
    con.move_velocity(1, 2, 0.0, False)
    print("Rover: track velocity (1, 2, 0.0)")
    time.sleep(5)

    # attitude guidance cmd
    print("Commander interface: start attitude control mode")
    print("set forward driving")
    con.set_yaw_and_throttle(3.14/2, 0.6)
    con.set_forward_driving()
    print("Rover: forward driving, target yaw pi/2, throttle 0.6")
    time.sleep(5)
    con.set_yaw_and_throttle(3.14/2, 0.6)
    con.set_backward_driving()
    print("Rover: backward driving, target yaw pi/2, throttle 0.6")
    time.sleep(5)

    # direct actuator control cmd
    print("Commander interface: start actuator control mode")
    con.set_steering_servo_and_throttle(0.5, 0.3)
    con.set_forward_driving()
    print("Rover: forward driving, steering input 0.5, throttle 0.3")
    time.sleep(5)
    con.set_steering_servo_and_throttle(0.5, 0.3)
    con.set_backward_driving()
    print("Rover: backward driving, steering input 0.5, throttle 0.3")

    #disarm for safety at the end
    print("Commander interface: disarm vehicle, mission completed")
    con.disarm()


