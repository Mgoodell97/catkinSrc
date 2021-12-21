#!/usr/bin/env python3

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import math
import numpy as np
import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils.callbacks import Caller

#---------------Global Variables------------
global crazyflie1_pose
crazyflie1_pose = PoseStamped()

global crazyflie2_pose
crazyflie2_pose = PoseStamped()

global crazyflie3_pose
crazyflie3_pose = PoseStamped()
#-------------Functions --------------------
def crazyflie1_pose_cb(pose_cb_msg):
    global crazyflie1_pose
    crazyflie1_pose = pose_cb_msg

def crazyflie2_pose_cb(pose_cb_msg):
    global crazyflie2_pose
    crazyflie2_pose = pose_cb_msg

def crazyflie3_pose_cb(pose_cb_msg):
    global crazyflie3_pose
    crazyflie3_pose = pose_cb_msg



def main():
    # Initalize node
    rospy.init_node('inidividualControl')
    print('Starting Hover ')

    #Get parameters
    # CFID=rospy.get_param("CF_ID")
    #zd = rospy.get_param("z_desired")
    xd1 = 2.6 #1.8
    yd1 = 0.6#.6
    zd1 = 0.25 #.75

    xd2 = 2.3 #1.8
    yd2 = 0.75#.6
    zd2 = 0.75 #.75

    xd3= 1.8
    yd3 = .6
    zd3 = 0.75





 #Set up subscribers
    rospy.Subscriber("/mocap_node/Crazyflie_1/pose", PoseStamped, crazyflie1_pose_cb)
    rospy.Subscriber("/mocap_node/Crazyflie_2/pose", PoseStamped, crazyflie2_pose_cb)
    rospy.Subscriber("/mocap_node/Crazyflie_3/pose", PoseStamped, crazyflie3_pose_cb)
    local_pose_pub = rospy.Publisher('Position', Pose, queue_size=10)
    Pose_pub = Pose()
    Pose_pub.position.x = 0 #thrust
    Pose_pub.position.y = 0 #error
    Pose_pub.position.z = 0 #z position

#Initialize Low Level Drivers
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers(enable_debug_driver=False)

#Initialize URI
    URI1 = 'radio://0/80/2M/E7E7E7E701'
    cf1 = Crazyflie()
    connected1 = Caller()
    cf1.connected.add_callback(connected1)
    cf1.open_link(URI1)
    cf1.commander.send_setpoint(0,0,0, 0)

    # URI2 = 'radio://0/80/2M/E7E7E7E702'
    # cf2 = Crazyflie()
    # connected2 = Caller()
    # cf2.connected.add_callback(connected1)
    # cf2.open_link(URI2)
    # cf2.commander.send_setpoint(0,0,0, 0)
    #
    URI3 = 'radio://0/80/2M/E7E7E7E703'
    cf3 = Crazyflie()
    connected3 = Caller()
    cf3.connected.add_callback(connected3)
    cf3.open_link(URI3)
    cf3.commander.send_setpoint(0,0,0, 0)




    # Set up publishers
    #Initialize Variables
    global rate
    roll = 0.0
    pitch =0.0
    yaw=0.0
    #rate = rospy.Rate(120)

    z_error1i=0
    # z_error2i=0
    z_error3i=0
    y_error1i = 0
    # y_error2i = 0
    y_error3i = 0
    x_error1i = 0
    # x_error2i = 0
    x_error3i = 0

    #Controller Gains
    #Kd = 25.0 * (10**5)
    Kd = 0
    Kp = .45 * (10**5)

    KpXY = 10
    #KdXY = 1200
    KdXY =0


    while not rospy.is_shutdown():
        print("In while loop")
        #for _ in range(1000):
        print("for loop")
        x_cf1 = crazyflie1_pose.pose.position.x
        y_cf1 = crazyflie1_pose.pose.position.y
        z_cf1 = crazyflie1_pose.pose.position.z

        # x_cf2 = crazyflie2_pose.pose.position.x
        # y_cf2 = crazyflie2_pose.pose.position.y
        # z_cf2 = crazyflie2_pose.pose.position.z
        #
        x_cf3 = crazyflie3_pose.pose.position.x
        y_cf3 = crazyflie3_pose.pose.position.y
        z_cf3 = crazyflie3_pose.pose.position.z

        print("CF1:",x_cf1,y_cf1,z_cf1)
        # print("CF2:",x_cf2,y_cf2,z_cf2)
        print("CF3:",x_cf3,y_cf3,z_cf3)

        z_error1 = zd1 - z_cf1
        z_error3 = zd3 - z_cf3
        y_error1 = yd1 - y_cf1
        y_error3 = yd3 - y_cf3
        x_error1 = xd1 - x_cf1
        x_error3 = xd3 - x_cf3

        # z_error2 = zd2 - z_cf2
        # y_error2 = yd2 - y_cf2
        # x_error2 = xd2 - x_cf2

        thrust1 = 34000*1 +  (Kp * z_error1 + Kd * (z_error1-z_error1i))*1
        # thrust2 = 10000*0  +  (Kp * z_error2 + Kd * (z_error2-z_error2i))*0
        thrust3 = 10000* 0 +  (Kp * z_error3 + Kd * (z_error3-z_error3i))*0

        pitch1 = (KpXY * y_error1 + KdXY * (y_error1-y_error1i))*1
        # pitch2 = (KpXY * y_error2 + KdXY * (y_error2-y_error2i))*0
        pitch3 = (KpXY * y_error3 + KdXY * (y_error3-y_error3i))*0

        roll1 = (KpXY * x_error1 + KdXY * (x_error1-x_error1i))*1
        # roll2 = (KpXY * x_error2 + KdXY * (x_error2-x_error2i))*0
        roll3 = (KpXY * x_error3 + KdXY * (x_error3-x_error3i))*0

        # print(thrust3)
        # print("z error", z_error3)
        #Print Statements
        # print("y1", y_cf1)
        # print("pitch1", pitch1)
        # print("y3", y_cf3)
        # print("pitch3", pitch3)

        if thrust1 >= 60000:
            thrust1 = 60000
        elif (thrust1 <= 10001):
            thrust1 = 10001

        # if thrust2 >= 60000:
        #     thrust2 = 60000
        # elif (thrust2 <= 10001):
        #     thrust2 = 10001
        #
        if thrust3 >= 60000:
            thrust3 = 60000
        elif (thrust3 <= 10001):
            thrust3 = 10001

        # if pitch1 >= 500:
        #     pitch1 = 500`
        # elif pitch1 <= 0:
        #     pitch1 = 0

        # cf1.commander.send_setpoint(0,0,0,0)
        # cf3.commander.send_setpoint(0,0,0,0)

        cf1.commander.send_setpoint(math.floor(roll1), math.floor(pitch1), yaw, math.floor(thrust1))
        # cf2.commander.send_setpoint(math.floor(roll2), math.floor(pitch2), yaw, math.floor(thrust2))
        cf3.commander.send_setpoint(math.floor(roll3), math.floor(pitch3), yaw, math.floor(thrust3))

        # cf1.commander.send_setpoint(0,0,0, 20000*1)
        # cf2.commander.send_setpoint(0,0,0, 20000*0)
        # cf3.commander.send_setpoint(0,0,0, 20000*0)
        #time.sleep()

        z_error1i = z_error1
        # z_error2i = z_error2
        z_error3i = z_error3
        y_error1i = y_error1
        # y_error2i = y_error2
        y_error3i = y_error3
        x_error1i = x_error1
        # x_error2i = x_error2
        x_error3i = x_error3

        Pose_pub.position.x = thrust1 #thrust
        Pose_pub.position.y = x_error1 #error
        Pose_pub.position.z = y_error1 #z position

        local_pose_pub.publish(Pose_pub)


        # cf1.close_link()
        # cf2.close_link()
        # cf3.close_link()
        # rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
