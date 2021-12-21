#!/usr/bin/env python3

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils.callbacks import Caller

#---------------Global Variables------------
global crazyflie_pose
crazyflie_pose = PoseStamped()
#-------------Functions --------------------
def crazyflie_pose_cb(pose_cb_msg):
    global crazyflie_pose
    crazyflie_pose = pose_cb_msg

def capFunc(currentValue, newMin, newMax):
    if(currentValue > newMax):
       currentValue = newMax
    if currentValue < newMin:
       currentValue = newMin
    return currentValue


def main():
    # Initalize node
    rospy.init_node('hoverControl')
    print('Starting Hover ')

    #Get parameters
    CFID=rospy.get_param("CF_ID")
    zd =

    # Set up subscriptions
    firstString = "/mocap_node/Crazyflie_"
    secondString = str(CFID)
    thirdString = "/pose"
    fullString = firstString + secondString + thirdString #this allows each robot to know itself
    rospy.Subscriber(fullString, PoseStamped, crazyflie_pose_cb)

    #Initialize crazyflie stuff

    string = 'radio://0/80/2M/E7E7E7E70'
    if (CFID ==1):
        URI1 = string + secondString
        print("URI1:", URI1)
    if (CFID ==3):
        URI3 = string + secondString
        print("URI3:", URI3)




    logging.basicConfig(level=logging.ERROR)

    # Called when the link is established and the TOCs (that are not
    # cached) have been downloaded
    connected = Caller()

    cflib.crtp.init_drivers(enable_debug_driver=False)

    if (CFID == 1):
        cf1 = Crazyflie()

        cf1.connected.add_callback(connected)

        cf1.open_link(URI1)

        cf1.commander.send_setpoint(0,0,0, 0)
    elif (CFID == 3):
        cf3 = Crazyflie()

        cf3.connected.add_callback(connected)

        cf3.open_link(URI3)

        cf3.commander.send_setpoint(0,0,0, 0)


    # Set up publishers


    #Initialize Variables
    global rate
    roll = 0.0
    pitch =0.0
    yaw=0.0
    rate = rospy.Rate(20)

    z_errori=0
    Kd = 1.5 * (10**5)
    Kp = .2 * (10**5)


    while not rospy.is_shutdown():
        print("In while loop",CFID)
        for _ in range(100):
            z=crazyflie_pose.pose.position.z
            z_error = zd - z

            thrust = 40000 +  Kp * z_error + Kd * (z_error-z_errori)

            if thrust >= 60000:
                thrust = 60000
            elif (thrust <= 10001):
                thrust = 10001
            if (CFID == 1):
                cf1.commander.send_setpoint(roll, pitch, yaw, math.floor(thrust))
            elif (CFID == 3):
                cf3.commander.send_setpoint(roll, pitch, yaw, math.floor(thrust))

            time.sleep(0.1)

            z_errori = z_error
            print ("Z",z,CFID)
            print ("Zd",zd, CFID)
            print("Thrust:", thrust, CFID)

        if (CFID == 1):
            cf1.close_link()
        elif (CFID == 3):
            cf3.close_link()
        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
