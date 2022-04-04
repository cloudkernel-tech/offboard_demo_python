#!/usr/bin/env python3

# Control Interface for Kerloud Autocar python API
# More information can be referred in: https://kerloud-autocar.readthedocs.io/
# All rights reserved for Cloudkernel Technologies (Shenzhen) Co., Ltd.
# Author: cloudkerneltech@gmail.com

import rospy
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget, ExtendedState, Thrust, ActuatorControl
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String, Bool

import time
from pyquaternion import Quaternion
import math
import threading

import argparse


# constant definitions for Kerloud Autocar
MAV_CMD_SET_ROVER_FORWARD_REVERSE_DRIVING = 3101 # cmd ID to set forward/backward driving state
MAV_FRAME_LOCAL_ENU = 4

LANDED_STATE_UNDEFINED = 0
LANDED_STATE_ON_GROUND = 1
LANDED_STATE_IN_AIR = 2
LANDED_STATE_TAKEOFF = 3
LANDED_STATE_LANDING = 4

# command state for kerloud autocar
POSITION_ROVER_COMMAND_MODE = 0
VELOCITY_ROVER_COMMAND_MODE = 1
ATTITUDE_ROVER_COMMAND_MODE = 2
ACTUATOR_CONTROL_ROVER_COMMAND_MODE = 3

ROVER_FORWARD_DRIVING_STATE = True
ROVER_BACKWARD_DRIVING_STATE = False

class Px4Controller:

    def __init__(self):
        self.imu = None
        self.extended_state = None
        self.local_pose = None
        self.local_velocity = None
        self.current_heading = None

        self.cur_pos_target = None   # current position target
        self.cur_vel_target = None   # current velocity target
        self.cur_attitude_target = None # current attitude target
        self.cur_thrust_target= None # desired thrust for low level autopilot
        self.cur_actuator_control_target = None # desired actuator control for low level autopilot

        self.arm_state = False       # flag to indicate that the vehicle is armed
        self.offboard_state = False  # flag to indicate that the vehicle is in offboard mode
        self.landed_state = "UNDEFINED"
        self.cmd_mode = POSITION_ROVER_COMMAND_MODE # current command mode

        # arm/disarm request variables
        self.flag_arm_req = False
        self.desired_armed_state = True
        self.last_arm_call_timestamp = 0

        # forward/backward driving request variables
        self.flag_set_driving_state = False  # true: set driving direction cmd is activated
        self.forward_driving_state = True   #true: rover is in forward driving state, backward driving is only applicable in attitude and actuator control modes
        self.desired_driving_state = ROVER_FORWARD_DRIVING_STATE  # desired driving state, true: forward driving
        self.last_set_driving_state_timestamp = 0
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_velocity_sub = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.local_velocity_callback)
        self.mavros_state_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        self.extended_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, self.extendedstate_callback)

        self.set_target_position_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, self.set_target_position_callback)
        self.set_target_velocity_sub = rospy.Subscriber("gi/set_pose/velocity", Vector3Stamped, self.set_target_velocity_callback)
        self.set_target_attitude_sub = rospy.Subscriber("gi/set_pose/attitude", PoseStamped, self.set_target_attitude_callback)
        self.set_thrust_sub = rospy.Subscriber("gi/set_thrust", Thrust, self.set_thrust_callback)
        self.set_actuator_control_sub = rospy.Subscriber("gi/set_act_control", ActuatorControl, self.set_actuator_control_callback)

        self.custom_activity_sub = rospy.Subscriber("gi/set_activity/type", String, self.custom_activity_callback)
        self.driving_state_sub = rospy.Subscriber('gi/set_driving_state', Bool, self.set_driving_state_callback)

        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=5) # position & velocity command modes
        self.attitude_target_pub = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=2)
        self.thrust_target_pub = rospy.Publisher('mavros/setpoint_attitude/thrust', Thrust, queue_size=2)
        self.actuator_control_target_pub = rospy.Publisher('mavros/actuator_control', ActuatorControl, queue_size=2)

        self.core_ready_pub = rospy.Publisher('gi/core_ready', Bool, queue_size=2)
        self.landedstate_pub = rospy.Publisher('gi/landed_state', String, queue_size=2)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.cmdService= rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        print("Kerloud python API control interface initialized!")


    '''entry function for core loop'''
    def start(self, flag_simulation_mode=False):
        rospy.init_node("offboard_node")
        rate = rospy.Rate(30)  # 30Hz

        for i in range(10):
            if self.current_heading is not None:
                break
            else:
                print("Control interface: waiting for initialization.")
                time.sleep(0.5)

        self.cur_pos_target = self.construct_position_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y)

        '''arm and set offboard automatically in simulation mode'''
        if flag_simulation_mode:
            print("setting arm and offboard in simulation mode...")

            while True:
                self.local_target_pub.publish(self.cur_pos_target)
                self.arm()
                self.offboard()
                if self.arm_state and self.offboard_state:
                    print("vehicle is armed and set in offboard mode")
                    break

                time.sleep(0.1)


        else:
            print("Control interface: starting in real test...")

        '''
        main ROS thread
        '''
        while not rospy.is_shutdown():

            # publications

            if self.cmd_mode == POSITION_ROVER_COMMAND_MODE:
                self.local_target_pub.publish(self.cur_pos_target)
            elif self.cmd_mode == VELOCITY_ROVER_COMMAND_MODE:
                self.local_target_pub.publish(self.cur_vel_target)
            elif self.cmd_mode == ATTITUDE_ROVER_COMMAND_MODE:
                self.attitude_target_pub.publish(self.cur_attitude_target)
                self.thrust_target_pub.publish(self.cur_thrust_target)
            elif self.cmd_mode == ACTUATOR_CONTROL_ROVER_COMMAND_MODE:
                self.actuator_control_target_pub.publish(self.cur_actuator_control_target)

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

            # response to forward/backward driving request
            if self.flag_set_driving_state:
                if time.time() - self.last_set_driving_state_timestamp > 1.0:
                    if self.desired_driving_state == ROVER_FORWARD_DRIVING_STATE:
                        self.set_driving_state(1)
                    else:
                        self.set_driving_state(0)

                    self.last_set_driving_state_timestamp = time.time()


            rate.sleep()


    ##### Key functions #####
    '''contruct 2D position target for Kerloud autocar'''
    """note that yaw control is not accessible here for nonholonomic vehicle"""
    def construct_position_target(self, x, y):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = MAV_FRAME_LOCAL_ENU #local ENU frame id, mavlink MAV_FRAME definition

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = self.local_pose.pose.position.z # use z measurement here

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE

        return target_raw_pose



    def construct_velocity_target(self, vx, vy):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = MAV_FRAME_LOCAL_ENU  #local ENU frame id, mavlink MAV_FRAME definition
        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = 0

        target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE

        return target_raw_pose


    ##### Callback functions #####
    def local_pose_callback(self, msg):
        self.local_pose = msg

    def local_velocity_callback(self, msg):
        self.local_velocity = msg

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


    def set_target_position_callback(self, msg):
        print("Control interface: new position target received!")
        self.cmd_mode = POSITION_ROVER_COMMAND_MODE

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

            ENU_X, ENU_Y, ENU_Z = self.convert_pos_FLU2ENU(msg)

            self.cur_pos_target = self.construct_position_target(ENU_X, ENU_Y)

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

            self.cur_pos_target = self.construct_position_target(msg.pose.position.x, msg.pose.position.y)


    def set_target_velocity_callback(self, msg):
        print("Control interface: new velocity target received!")
        self.cmd_mode = VELOCITY_ROVER_COMMAND_MODE

        if msg.header.frame_id == 'base_link':
            print("body FLU frame")
            ENU_vx, ENU_vy, ENU_Z = self.convert_vel_FLU2ENU(msg)
            self.cur_vel_target = self.construct_velocity_target(ENU_vx, ENU_vy)

        else:
            print("local ENU frame")
            self.cur_vel_target = self.construct_velocity_target(msg.vector.x, msg.vector.y)


    def set_target_attitude_callback(self, msg):
        print("Control interface: new attitude target received!")
        self.cmd_mode = ATTITUDE_ROVER_COMMAND_MODE
        self.cur_attitude_target = msg


    def set_thrust_callback(self, msg):
        print("Control interface: new thrust target received!")
        self.cur_thrust_target = msg

    def set_actuator_control_callback(self, msg):
        self.cmd_mode = ACTUATOR_CONTROL_ROVER_COMMAND_MODE
        print("Control interface: new actuator control target received!")
        self.cur_actuator_control_target = msg


    '''
     Receive A Custom Activity
     '''

    def custom_activity_callback(self, msg):
        print("Control interface: custom activity received:", msg.data)

        if msg.data == "ARM":
            print("Arm requested!")
            self.flag_arm_req = True
            self.desired_armed_state = True
        elif msg.data == "DISARM":
            print("Disarm requested!")
            self.flag_arm_req = True
            self.desired_armed_state = False
        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")


    '''callback for forward/backward driving command in attitude and actuator control command states'''
    def set_driving_state_callback(self, msg):
        print("Control interface: set driving state cmd received")

        if self.cmd_mode == ATTITUDE_ROVER_COMMAND_MODE or self.cmd_mode == ACTUATOR_CONTROL_ROVER_COMMAND_MODE:
            if msg.data == True:
                self.flag_set_driving_state = True
                self.desired_driving_state = ROVER_FORWARD_DRIVING_STATE
            else:
                self.flag_set_driving_state = True
                self.desired_driving_state = ROVER_BACKWARD_DRIVING_STATE
        else:
            print("Control interface: driving state cmd not supported in position/velocity guidance, forward driving by default")

    ####### Actions for Kerloud vehicle operation ######
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

    '''set desired driving state, state=1 for forward driving, state=0 for backward driving'''
    def set_driving_state(self, state=1):
        if self.cmdService(command=MAV_CMD_SET_ROVER_FORWARD_REVERSE_DRIVING, confirmation=0, param1=state):
            self.flag_set_driving_state = False # clear the req flag in main loop
            return True
        else:
            print("Vehicle set driving state service call failed")
            return False



    ##### Supporting functions ######
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

    '''convert position in FLU to ENU'''
    def convert_pos_FLU2ENU(self, msg):
        ENU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(self.current_heading)
        ENU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(self.current_heading)
        ENU_x = ENU_x + self.local_pose.pose.position.x
        ENU_y = ENU_y + self.local_pose.pose.position.y
        ENU_z = msg.pose.position.z

        return ENU_x, ENU_y, ENU_z


    '''convert velocity in FLU to ENU'''
    def convert_vel_FLU2ENU(self, msg):

        ENU_vx = msg.vector.x * math.cos(self.current_heading) - msg.vector.y * math.sin(self.current_heading)
        ENU_vy = msg.vector.x * math.sin(self.current_heading) + msg.vector.y * math.cos(self.current_heading)
        ENU_vz = msg.vector.z

        return ENU_vx, ENU_vy, ENU_vz

    '''
    cur_p : poseStamped
    target_p: positionTarget
    threshold: threshold for position reach condition, unit: m
    '''
    def check_position_reach_condition(self, cur_p, target_p, threshold=0.1):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)

        if (delta_x + delta_y < threshold):
            return True
        else:
            return False


def main():

    parser = argparse.ArgumentParser(description='python mavros main entry function')
    parser.add_argument('--sim', type=bool, action="store", default=False, help='Simulation flag: True for software in the loop simulation, false for real tests')
    args = parser.parse_args()

    con = Px4Controller()
    con.start(args.sim)     # use for simulation by default



if __name__ == '__main__':
    main()



