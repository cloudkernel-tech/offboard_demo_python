#!/usr/bin/env python

# Control Interface for Kerloud Autocar python API
# More information can be referred in: https://kerloud-autocar.readthedocs.io/
# All rights reserved for Cloudkernel Technologies (Shenzhen) Co., Ltd.
# Author: cloudkerneltech@gmail.com

import rospy
from mavros_msgs.msg import State, PositionTarget, ExtendedState, Thrust, ActuatorControl
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String, Bool

import time
from pyquaternion import Quaternion
import math
import threading

# constant definitions for Kerloud Autocar
MAV_CMD_SET_ROVER_FORWARD_REVERSE_DRIVING = 3101 # cmd ID to set forward/backward driving state

LANDED_STATE_UNDEFINED = 0
LANDED_STATE_ON_GROUND = 1
LANDED_STATE_IN_AIR = 2
LANDED_STATE_TAKEOFF = 3
LANDED_STATE_LANDING = 4

# command state for kerloud autocar
POSITION_COMMAND_MODE = 0
VELOCITY_COMMAND_MODE = 1
ATTITUDE_COMMAND_MODE = 2
ACTUATOR_CONTROL_COMMAND_MODE = 3



class Px4Controller:

    def __init__(self):
        self.imu = None
        self.extended_state = None
        self.local_pose = None
        self.current_heading = None
        self.cur_pos_target = None   # current position target

        self.arm_state = False       # flag to indicate that the vehicle is armed
        self.offboard_state = False  # flag to indicate that the vehicle is in offboard mode
        self.landed_state = "UNDEFINED"
        self.cmd_mode = POSITION_COMMAND_MODE # current command mode

        # arm/disarm request variables
        self.flag_arm_req = False
        self.desired_armed_state = True
        self.last_arm_call_timestamp = 0

        # forward/backward driving request variables



        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_state_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        self.extended_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, self.extendedstate_callback)

        self.set_target_position_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, self.set_target_position_callback)
        self.set_target_velocity_sub = rospy.Subscriber("gi/set_pose/velocity", Twist, self.set_target_velocity_callback)
        self.set_target_attitude_sub = rospy.Subscriber("gi/set_pose/attitude", PoseStamped, self.set_target_attitude_callback)
        self.set_thrust_sub = rospy.Subscriber("gi/set_thrust", Thrust, self.set_thrust_callback)
        self.set_actuator_control_sub = rospy.Subscriber("gi/set_act_control", Thrust, self.set_actuator_control_callback)

        self.custom_activity_sub = rospy.Subscriber("gi/set_activity/type", String, self.custom_activity_callback)

        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.core_ready_pub = rospy.Publisher('gi/core_ready', Bool, queue_size=2)
        self.landedstate_pub = rospy.Publisher('gi/landed_state', String, queue_size=2)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.cmdService= rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        print("Px4 Controller Initialized!")


    '''entry function for core loop'''
    def start(self, flag_simulation_mode=False):
        rospy.init_node("offboard_node")
        rate = rospy.Rate(30)  # 30Hz

        for i in range(10):
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        self.cur_pos_target = self.construct_position_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     self.current_heading)

        '''arm and set offboard automatically in simulation mode'''
        if flag_simulation_mode:
            print("setting arm and offboard in simulation mode...")

            for i in range(100):
                self.local_target_pub.publish(self.cur_pos_target)
                self.arm_state = self.arm()
                self.offboard_state = self.offboard()

                if self.arm_state and self.offboard_state:
                    break

                time.sleep(0.05)
        else:
            print("Caution: starting in real test...")

        '''
        main ROS thread
        '''
        while not rospy.is_shutdown():
            # publications
            self.local_target_pub.publish(self.cur_pos_target)
            self.landedstate_pub.publish(String(self.landed_state))

            # publish flag to indicate the vehicle is armed and in offboard
            if self.arm_state and self.offboard_state:
                self.core_ready_pub.publish(True)
            else:
                self.core_ready_pub.publish(False)

            # response to arm/disarm request
            if self.flag_arm_req:
                if time.time() - self.last_arm_call_timestamp > 1.0:
                    if self.desired_armed_state:
                        self.arm()
                    else:
                        self.disarm()
                    self.last_arm_call_timestamp = time.time()

                if self.arm_state == self.desired_armed_state:
                    self.flag_arm_req = False

            rate.sleep()


    '''contruct position target for autopilot'''
    def construct_position_target(self, x, y, z, yaw, yaw_rate = 0):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 4 #local ENU frame id, mavlink MAV_FRAME definition

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE + PositionTarget.IGNORE_YAW_RATE

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate  # feedward yaw speed here if not ignored

        return target_raw_pose



    '''
    cur_p : poseStamped
    target_p: positionTarget
    '''
    def position_distance(self, cur_p, target_p, threshold=0.1):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

        if (delta_x + delta_y + delta_z < threshold):
            return True
        else:
            return False


    def local_pose_callback(self, msg):
        self.local_pose = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode
        self.arm_state = msg.armed
        self.offboard_state = (msg.mode == "OFFBOARD")

    def imu_callback(self, msg):
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation)

    '''callback for extended state subscription'''
    def extendedstate_callback(self, msg):
        self.extended_state = msg

        if self.extended_state.landed_state == LANDED_STATE_UNDEFINED:
            self.landed_state = "UNDEFINED"
        elif self.extended_state.landed_state == LANDED_STATE_ON_GROUND:
            self.landed_state = "ON_GROUND"
        elif self.extended_state.landed_state == LANDED_STATE_IN_AIR:
            self.landed_state == "IN_AIR"
        elif self.extended_state.landed_state == LANDED_STATE_TAKEOFF:
            self.landed_state == "TAKEOFF"
        elif self.extended_state.landed_state == LANDED_STATE_LANDING:
            self.landed_state == "LANDING"


    '''convert position in FLU to ENU'''
    def FLU2ENU(self, msg):
        ENU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(self.current_heading)
        ENU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(self.current_heading)
        ENU_z = msg.pose.position.z
        return ENU_x, ENU_y, ENU_z


    def set_target_position_callback(self, msg):
        print("New position target received!")

        if msg.header.frame_id == 'base_link':
            '''
            BODY_FLU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            print("body FLU frame")

            ENU_X, ENU_Y, ENU_Z = self.FLU2ENU(msg)

            ENU_X = ENU_X + self.local_pose.pose.position.x
            ENU_Y = ENU_Y + self.local_pose.pose.position.y
            ENU_Z = ENU_Z + self.local_pose.pose.position.z

            self.cur_pos_target = self.construct_position_target(ENU_X,
                                                         ENU_Y,
                                                         ENU_Z,
                                                         self.current_heading)


        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            print("local ENU frame")

            self.cur_pos_target = self.construct_position_target(msg.pose.position.x,
                                                         msg.pose.position.y,
                                                         msg.pose.position.z,
                                                         self.current_heading)

    '''
     Receive A Custom Activity
     '''

    def custom_activity_callback(self, msg):
        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND":
            print("LANDING!")
            self.cur_pos_target = self.construct_position_target(self.local_pose.pose.position.x,
                                                         self.local_pose.pose.position.y,
                                                         -5.0,
                                                         self.current_heading)
        elif msg.data == "ARM":
            print("Arm requested!")
            self.flag_arm_req = True
            self.desired_armed_state = True
        elif msg.data == "DISARM":
            print("Disarm requested!")
            self.flag_arm_req = True
            self.desired_armed_state = False
        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")


    '''
    return yaw from current IMU
    '''
    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarm failed!")
            return False


    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vehicle set offboard failed")
            return False


if __name__ == '__main__':
    con = Px4Controller()
    con.start(False)# use for real test
    #con.start(True) # use for simulation

