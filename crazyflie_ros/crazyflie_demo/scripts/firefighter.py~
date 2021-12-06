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

    # takeoff    
    # zDistance = 0.4
    # time_range = 1 + int(10*zDistance/0.4)
    # while not rospy.is_shutdown():
    #     for y in range(time_range):
    #         msgHov.vx = 0.0
    #         msgHov.vy = 0.0
    #         msgHov.yawrate = 0.0
    #         msgHov.zDistance = y / 25.0
    #         msgHov.header.seq += 1
    #         msgHov.header.stamp = rospy.Time.now()
    #         pubHov.publish(msgHov)
    #         rate.sleep()
    #     for y in range(20):
    #         msgHov.vx = 0.0
    #         msgHov.vy = 0.0
    #         msgHov.yawrate = 0.0
    #         msgHov.zDistance = zDistance
    #         msgHov.header.seq += 1
    #         msgHov.header.stamp = rospy.Time.now()
    #         pubHov.publish(msgHov)
    #         rate.sleep()
    #     break

    # Stay
    while not rospy.is_shutdown():
        for y in range(40):
            msgPos.x = 0.0
            msgPos.y = 0.0
            msgPos.yaw = 0.0
            msgPos.z = 1.0
            msgPos.header.seq += 1
            msgPos.header.stamp = rospy.Time.now()
            # rospy.loginfo(now)
            pubPos.publish(msgPos)
            if rospy.is_shutdown():
                break
            rate.sleep()
        for y in range(200):
            msgPos.z = 1.0
            # 4 Circles
            # msgPos.x = 0.3*sin(y/200.0*2*pi*4.0)
            # msgPos.y = 0.3*cos(y/200.0*2*pi*4.0)
            # Figure 8
            msgPos.x = 0.5*cos(y/200.0*2*pi*4.0)
            msgPos.y = 0.5*sin(y/200.0*2*pi*4.0)*cos(y/200.0*2*pi*4.0)
            # a = .2
            # msgPos.x = a*(1+cos(y/200.0*2*pi*4.0))
            # msgPos.y = a*sin(y/200.0*2*pi*4.0)
            # msgPos.z = 2.0*a*sin(0.5*y/200.0*2*pi*4.0) + 2.0*a + .1
            # msgPos.x = 0
            # msgPos.y = 0
            msgPos.yaw = 0.0
            msgPos.header.seq += 1
            msgPos.header.stamp = rospy.Time.now()
            # rospy.loginfo(now)
            pubPos.publish(msgPos)
            if rospy.is_shutdown():
                break
            rate.sleep()
        break

    # # go to x: 0.2 y: 0.2
    # start = rospy.get_time()
    # while not rospy.is_shutdown():
    #     msgPos.x = 0.2
    #     msgPos.y = 0.2
    #     msgPos.yaw = 0.0
    #     msgPos.z = 0.4
    #     now = rospy.get_time()
    #     if (now - start > 10.0):
    #         break
    #     msgPos.header.seq += 1
    #     msgPos.header.stamp = rospy.Time.now()
    #     rospy.loginfo("sending...")
    #     rospy.loginfo(msgPos.x)
    #     rospy.loginfo(msgPos.y)
    #     rospy.loginfo(msgPos.z)
    #     rospy.loginfo(msgPos.yaw)
    #     pub.publish(msgPos)
    #     rate.sleep()

    # land, spend 1 secs
    start = rospy.get_time()
    while not rospy.is_shutdown():
        msgPos.x = 0.0
        msgPos.y = 0.0
        msgPos.z = 0.0
        msgPos.yaw = 0.0
        now = rospy.get_time()
        if (now - start > 1.0):
            break
        msgPos.header.seq += 1
        msgPos.header.stamp = rospy.Time.now()
        rospy.loginfo("sending...")
        rospy.loginfo(msgPos.x)
        rospy.loginfo(msgPos.y)
        rospy.loginfo(msgPos.z)
        rospy.loginfo(msgPos.yaw)
        pubPos.publish(msgPos)
        rate.sleep()

    stop_pub.publish(stop_msg)
