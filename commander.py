#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String, Bool
import time
import math


class Commander:
    def __init__(self):
        # current flying rover mode
        self.flyingrover_mode = "ROVER"
        # flag to indicate that the mavros core is ready to work
        self.flag_core_ready = False
        # landed state
        self.landed_state = "UNDEFINED"

        rospy.init_node("commander_node")
        rate = rospy.Rate(30)

        '''publications and subscriptions'''
        self.position_target_pub = rospy.Publisher('gi/set_pose/position', PoseStamped, queue_size=10)
        self.yaw_target_pub = rospy.Publisher('gi/set_pose/orientation', Float32, queue_size=10)
        self.custom_activity_pub = rospy.Publisher('gi/set_activity/type', String, queue_size=10)

        self.flyingrover_mode_sub = rospy.Subscriber("gi/flyingrove_mode", String, self.flyingrover_mode_callback)
        self.core_ready_sub = rospy.Subscriber("gi/core_ready", Bool, self.coreready_callback)
        self.landedstate_sub = rospy.Subscriber("gi/landed_state", String, self.landedstate_callback)

    # callback for current flying rover mode
    def flyingrover_mode_callback(self, msg):
        self.flyingrover_mode = msg.data

    # callback for core ready indication (armed and offboard)
    def coreready_callback(self, msg):
        self.flag_core_ready = msg.data

    # callback for landed state
    def landedstate_callback(self, msg):
        self.landed_state = msg.data

    # move to a target position
    def move(self, x, y, z, BODY_FLU=False):
        self.position_target_pub.publish(self.set_pose(x, y, z, BODY_FLU))

    # turn to a desired yaw angle
    def turn(self, yaw_degree):
        self.yaw_target_pub.publish(yaw_degree)

    # land at current position
    def land(self):
        self.custom_activity_pub.publish(String("LAND"))

    # hover at current position
    def hover(self):
        self.custom_activity_pub.publish(String("HOVER"))

    # arm the vehicle on land
    def arm(self):
        self.custom_activity_pub.publish(String("ARM"))

    # disarm the vehicle on land
    def disarm(self):
        self.custom_activity_pub.publish(String("DISARM"))

    # request transition to rover mode
    def transit_to_rover(self):
        self.custom_activity_pub.publish(String("TRANSIT_TO_ROVER"))

    # request transition to multicopter mode
    def transit_to_mc(self):
        self.custom_activity_pub.publish(String("TRANSIT_TO_MC"))

    # return to home position with defined height
    def return_home(self, height):
        self.position_target_pub.publish(self.set_pose(0, 0, height, False))

    # set waypoint pose
    def set_pose(self, x=0, y=0, z=2, BODY_FLU = False):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()

        # ROS uses ENU internally, so we will stick to this convention
        if BODY_FLU:
            pose.header.frame_id = 'base_link'

        else:
            pose.header.frame_id = 'map'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        return pose


if __name__ == "__main__":

    # the mission performs a square waypoint flight with position target defined in FLU frame
    con = Commander()
    time.sleep(2)

    # waiting for core to be ready
    while not con.flag_core_ready:
        print("waiting for the vehicle to be armed and in offboard")
        time.sleep(1)

    # rover mission round
    con.move(5, 0, 0)
    time.sleep(5)
    con.move(5, 5, 0)
    time.sleep(5)
    con.move(0, 5, 0)
    time.sleep(5)
    con.move(0, 0, 0)
    time.sleep(5)

    # request to transit to multicopter
    while not con.flyingrover_mode == "MC":
        print("request to transit to multicopter mode")
        con.transit_to_mc()
        time.sleep(2)

    # multicopter mission round
    con.move(5, 0, 5)
    time.sleep(5)
    con.move(5, 5, 5)
    time.sleep(5)
    con.move(0, 5, 5)
    time.sleep(5)
    con.move(0, 0, 5)
    time.sleep(5)
    con.land()
    time.sleep(5)

    #disarm for safety at the end
    con.disarm()





