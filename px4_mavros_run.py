#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State, PositionTarget, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandLong
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String, Bool

import time
from pyquaternion import Quaternion
import math
import threading

# constant definitions for flying rover state
FLYINGROVER_STATE_UNDEFINED = 0
FLYINGROVER_STATE_ROVER = 1
FLYINGROVER_STATE_MC = 2
FLYINGROVER_STATE_TRANSITION_TO_MC = 3
FLYINGROVER_STATE_TRANSITION_TO_ROVER = 4

MAV_CMD_DO_FLYINGROVER_TRANSITION = 3100


class Px4Controller:

    def __init__(self):
        self.imu = None
        self.extended_state = None
        self.local_pose = None
        self.current_heading = None
        self.local_enu_position = None
        self.cur_target_pose = None
        self.arm_state = False       # flag to indicate that the vehicle is armed
        self.offboard_state = False  # flag to indicate that the vehicle is in offboard mode

        # current flying rover mode: Rover or Multicopter
        self.current_fr_mode = "ROVER"
        self.flag_transition_req = False
        self.desired_fr_mode = "ROVER"
        self.last_transition_call_timestamp = 0

        # arm/disarm request flag
        self.flag_arm_req = False
        self.desired_armed_state = True
        self.last_arm_call_timestamp = 0

        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_state_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        self.extended_sub = rospy.Subscriber("/mavros/extended_state", ExtendedState, self.extendedstate_callback)

        self.set_target_position_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, self.set_target_position_callback)
        self.set_target_yaw_sub = rospy.Subscriber("gi/set_pose/orientation", Float32, self.set_target_yaw_callback)
        self.custom_activity_sub = rospy.Subscriber("gi/set_activity/type", String, self.custom_activity_callback)

        '''
        ros publishers
        '''
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.flyingrover_mode_pub = rospy.Publisher('gi/flyingrove_mode', String, queue_size=2)
        self.core_ready_pub = rospy.Publisher('gi/core_ready', Bool, queue_size=2)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.flyingroverTransitionService = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

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

        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     self.current_heading)

        '''arm and set offboard automatically in simulation mode'''
        if flag_simulation_mode:
            print("setting arm and offboard in simulation mode...")

            for i in range(100):
                self.local_target_pub.publish(self.cur_target_pose)
                self.arm_state = self.arm()
                self.offboard_state = self.offboard()

                if self.arm_state and self.offboard_state:
                    break

                time.sleep(0.05)

        '''
        main ROS thread
        '''
        while not rospy.is_shutdown():
            # publications
            self.local_target_pub.publish(self.cur_target_pose)
            self.flyingrover_mode_pub(self.current_fr_mode)

            # publish flag to indicate the vehicle is armed and in offboard
            if self.arm_state and self.offboard_state:
                self.core_ready_pub(True)
            else:
                self.core_ready_pub(False)

            # response to mode transition request
            if self.flag_transition_req:
                if time.time() - self.last_transition_call_timestamp>1.0:
                    if self.desired_fr_mode == "ROVER":
                        self.transit_to_rover()
                    elif self.desired_fr_mode == "MC":
                        self.transit_to_mc()
                    else:
                        print("Only ROVER and MC flying rover modes are supported!")

                    self.last_transition_call_timestamp = time.time()

                if self.current_fr_mode == self.desired_fr_mode:
                    self.flag_transition_req = False

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
    def construct_target(self, x, y, z, yaw, yaw_rate = 0):
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
        self.local_enu_position = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode
        self.arm_state = self.mavros_state.armed
        self.offboard_state = (self.mavros_state.mode == "OFFBOARD")

    def imu_callback(self, msg):
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation)

    '''callback for extended state subscription'''
    def extendedstate_callback(self, msg):
        self.extended_state = msg
        if self.extended_state.flyingrover_state == FLYINGROVER_STATE_UNDEFINED:
            self.current_fr_mode = "UNDEFINED"
        elif self.extended_state.flyingrover_state == FLYINGROVER_STATE_ROVER:
            self.current_fr_mode = "ROVER"
        elif self.extended_state.flyingrover_state == FLYINGROVER_STATE_MC:
            self.current_fr_mode = "MC"
        elif self.extended_state.flyingrover_state == FLYINGROVER_STATE_TRANSITION_TO_MC:
            self.current_fr_mode = "ROVER_TRANSITION_TO_MC"
        elif self.extended_state.flyingrover_state == FLYINGROVER_STATE_TRANSITION_TO_ROVER:
            self.current_fr_mode = "MC_TRANSITION_TO_ROVER"
        else:
            print("received flying rover state is not supported")

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

            self.cur_target_pose = self.construct_target(ENU_X,
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

            self.cur_target_pose = self.construct_target(msg.pose.position.x,
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
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                         self.local_pose.pose.position.y,
                                                         -5.0,
                                                         self.current_heading)
        elif msg.data == "HOVER":
            print("HOVERING!")
            self.hover()
        elif msg.data == "ARM":
            print("Arm requested!")
            self.flag_arm_req = True
            self.desired_armed_state = True
        elif msg.data == "DISARM":
            print("Disarm requested!")
            self.flag_arm_req = True
            self.desired_armed_state = False
        elif msg.data == "TRANSIT_TO_ROVER":
            print("Transition to Rover requested!")
            self.desired_fr_mode = "Rover"
            self.flag_transition_req = True
        elif msg.data == "TRANSIT_TO_MC":
            print("Transition to Multicopter requested!")
            self.desired_fr_mode = "MC"
            self.flag_transition_req = True
        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")


    def set_target_yaw_callback(self, msg):
        print("Received New Yaw Task!")

        yaw_rad = msg.data * math.pi / 180.0
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     yaw_rad)

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

    def transit_to_mc(self):
        if self.flyingroverTransitionService(command=MAV_CMD_DO_FLYINGROVER_TRANSITION, confirmation=0, param1=FLYINGROVER_STATE_MC):
            print("transition to mc service is called")
        else:
            print("transition to mc service call failed")

    def transit_to_rover(self):
        if self.flyingroverTransitionService(command=MAV_CMD_DO_FLYINGROVER_TRANSITION, confirmation=0, param1=FLYINGROVER_STATE_ROVER):
            print("transition to mc service is called")
        else:
            print("transition to mc service call failed")

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


    def hover(self):
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     self.current_heading)

if __name__ == '__main__':
    con = Px4Controller()
    con.start()

