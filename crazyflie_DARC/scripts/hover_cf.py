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
global crazyflie_pose
crazyflie_pose = PoseStamped()
#-------------Functions --------------------
def crazyflie_pose_cb(pose_cb_msg):
    global crazyflie_pose
    crazyflie_pose = pose_cb_msg

def main():
    #print(cflib.crtp)
    #cflib.crtp.init_drivers(enable_debug_driver=False)

    #time.sleep()
    #startTest = rospy.get_param("/startTest") #.75
    # while ((not rospy.is_shutdown() and not startTest) ):
    #    #rate.sleep()
    #    print("Waiting to startTest")
    #    startTest = rospy.get_param("/startTest");
    #    if startTest:
    #        break

    # Initalize node
    rospy.init_node('hoverControl')
    print('Starting Hover ')

    global rate
    rate = rospy.Rate(50)
    #Get parameters
    CFID= rospy.get_param("CF_ID")

    xd =0  #rospy.get_param("x_d") #1.8
    yd =0 #rospy.get_param("y_d")#.6
    zd =0 #rospy.get_param("z_d") #.75


    # Set up subscriptions
    firstString = "/mocap_node/Crazyflie_"
    secondString = str(CFID)
    thirdString = "/pose"
    fullString = firstString + secondString + thirdString #this allows each robot to know itself
    rospy.Subscriber(fullString, PoseStamped, crazyflie_pose_cb)

    # if (CFID == 1):
    #     cflib.crtp.init_drivers(enable_debug_driver=False)



    #Initialize crazyflie stuff

    string = 'radio://0/80/2M/E7E7E7E70'
    URI = string + secondString
    print("CFID", URI)

    # if (CFID ==1):
    #     URI1 = string + secondString
    #     print("URI1:", URI1)
    # if (CFID ==3):
    #     URI3 = string + secondString
    #     print("URI3:", URI3)




    #logging.basicConfig(level=logging.ERROR)
    #cflib.crtp.init_drivers(enable_debug_driver=False)

    # Called when the link is established and the TOCs (that are not
    # cached) have been downloaded
    #connected = Caller()



    # while ((not rospy.is_shutdown() and not startTest) ):
    #    #rate.sleep()
    #    print("Waiting to startTest")
    #    startTest = rospy.get_param("/startTest");
    #    if startTest:
    #        break

    cf = Crazyflie()


#    cf.connected.add_callback(connected)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf.open_link(URI)

    cf.commander.send_setpoint(0,0,0, 0)

    # if (CFID == 1):
    #     cf1 = Crazyflie()
    #
    #     cf1.connected.add_callback(connected)
    #
    #     cf1.open_link(URI1)
    #
    #     cf1.commander.send_setpoint(0,0,0, 0)
    # elif (CFID == 3):
    #     cf3 = Crazyflie()
    #
    #     cf3.connected.add_callback(connected)
    #
    #     cf3.open_link(URI3)
    #
    #     cf3.commander.send_setpoint(0,0,0, 0)


    # Set up publishers

    local_pose_pub = rospy.Publisher('Position', Pose, queue_size=10)
    Pose_pub = Pose()
    Pose_pub.position.x = 0 #thrust
    Pose_pub.position.y = 0 #error
    Pose_pub.position.z = 0 #z position


    #Initialize Variables

    roll = 0.0
    pitch =0.0
    yaw=0.0

    x_errori = 0
    y_errori = 0
    z_errori = 0

    #Controller Gains
    Kd = 25.0 * (10**5)
    Kp = .45 * (10**5)

    KpXY = 15
    KdXY = 1200


    while not rospy.is_shutdown():
        print("In while loop",CFID)
        x = crazyflie_pose.pose.position.x
        y = crazyflie_pose.pose.position.y
        z = crazyflie_pose.pose.position.z

        x_error = xd - x
        y_error = yd - y
        z_error = zd - z

        roll   = KpXY * x_error + KdXY * (x_error - x_errori)
        pitch  = KpXY * y_error + KdXY * (y_error - y_errori)
        thrust = 40000 +  (Kp * z_error + Kd * (z_error - z_errori))*1

        if thrust >= 60000:
            thrust = 60000
        elif (thrust <= 10001):
            thrust = 10001


        #cf.commander.send_setpoint(math.floor(roll), math.floor(pitch), yaw, math.floor(thrust))
        cf.commander.send_setpoint(0,0, yaw, math.floor(10001))

        x_errori = x_error
        y_errori = y_error
        z_errori = z_error

        print ("Z",z,CFID)
        print ("Zd",zd, CFID)
        print("Thrust:", thrust, CFID)

        Pose_pub.position.x = thrust #thrust
        Pose_pub.position.y = x_error #error
        Pose_pub.position.z = y_error #z position
        local_pose_pub.publish(Pose_pub)

        rate.sleep()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
