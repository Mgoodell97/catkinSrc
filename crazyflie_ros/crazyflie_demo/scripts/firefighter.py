#!/usr/bin/env python

import rospy
import tf
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from math import *
from numpy import *

if __name__ == '__main__':
    rospy.init_node('position', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")

    
    rate = rospy.Rate(10) # 10 hz

    name = "cmd_position"

    msgPos = Position()
    msgPos.header.seq = 0
    msgPos.header.stamp = rospy.Time.now()
    msgPos.header.frame_id = worldFrame
    msgPos.x = 0.0
    msgPos.y = 0.0
    msgPos.z = 0.0
    msgPos.yaw = 0.0

    msgHov = Hover()
    msgHov.header.seq = 0
    msgHov.header.stamp = rospy.Time.now()
    msgHov.header.frame_id = worldFrame
    msgHov.yawrate = 0

    pubPos = rospy.Publisher(name, Position, queue_size=1)
    pubHov = rospy.Publisher("cmd_hover", Hover, queue_size=1)
    

    stop_pub = rospy.Publisher("cmd_stop", Empty, queue_size=1)
    stop_msg = Empty()

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    update_params(["kalman/initialX"])
    rospy.sleep(0.1)
    update_params(["kalman/initialY"])
    rospy.sleep(0.1)
    rospy.set_param("kalman/resetEstimation", 1)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.1)
    rospy.set_param("kalman/resetEstimation", 0)
    update_params(["kalman/resetEstimation"])
    rospy.sleep(0.5)

    print "HERE!!!"
    
    # Main process
    while not rospy.is_shutdown():
        # Takeoff
        for y in range(20):
            msgPos.x = 0.0
            msgPos.y = 0.0
            msgPos.yaw = 0.0
            msgPos.z = 0.25
            msgPos.header.seq += 1
            msgPos.header.stamp = rospy.Time.now()
            # rospy.loginfo(now)
            pubPos.publish(msgPos)
            if rospy.is_shutdown():
                break
            rate.sleep()
        # Define forward turn and backward time
        T_forward = 40.0
        T_turn = 20.0
        T_backward = 40.0
        for y in range(int(T_forward+T_turn+T_backward)):
            msgPos.z = 0.25
            if y < T_forward:
                msgPos.x = -0.5*cos(y/T_forward*2.0*pi*0.5)+0.5;
                msgPos.yaw = 0.0
            elif y < T_forward+T_turn:
                msgPos.x = 1.0
                msgPos.yaw = (y-T_forward)/T_turn*180
            else:
                msgPos.x = 0.5*cos((y-T_forward-T_turn)/T_forward*2.0*pi*0.5)+0.5;
                msgPos.yaw = 180
            msgPos.y = 0
            msgPos.header.seq += 1
            msgPos.header.stamp = rospy.Time.now()
            # rospy.loginfo(now)
            pubPos.publish(msgPos)
            if rospy.is_shutdown():
                break
            rate.sleep()
        break

    # land, spend 1 secs
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msgPos.x = 0.0
        msgPos.y = 0.0
        msgPos.z = 0.0
        msgPos.yaw = 180.0
        now = rospy.get_time()
        if (now - start > 1.0):
            break
        msgPos.header.seq += 1
        msgPos.header.stamp = rospy.Time.now()
        #rospy.loginfo("sending...")
        #rospy.loginfo(msgPos.x)
        #rospy.loginfo(msgPos.y)
        #rospy.loginfo(msgPos.z)
        #rospy.loginfo(msgPos.yaw)
        pubPos.publish(msgPos)
        rate.sleep()

    stop_pub.publish(stop_msg)
