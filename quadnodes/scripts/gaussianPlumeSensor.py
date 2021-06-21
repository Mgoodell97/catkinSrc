#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos, sqrt, exp
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist #include <geometry_msgs/Twist.h>
from quadnodes.msg import gaussian #include <geometry_msgs/Twist.h>


##################
# Functions
##################

def gaussFunc(xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc + (zFunc**2)/DzFunc))
    return con * 1000 # convert from kg/m^3 to ppm

def getReading(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    # Rotate frame

    Stheta = sin(thetaFunc)
    Ctheta = cos(thetaFunc)

    xRobotRotated = (Ctheta  * xRobotDef + Stheta * yRobotDef + -Ctheta * xPlumeFunc - Stheta * yPlumeFunc)
    yRobotRotated = (-Stheta * xRobotDef + Ctheta * yRobotDef +  Stheta * xPlumeFunc - Ctheta * yPlumeFunc)

    if xRobotRotated <= 0:
        reading = 0
    else:
        reading = gaussFunc(xRobotRotated,yRobotRotated,zFunc,QFunc,vFunc,DyFunc,DzFunc)

    return reading

##################
# Global variables
##################

global UAV_pose

UAV_pose = PoseStamped()
tmpMsg = gaussian()

##################
# Callbacks
##################

def UAV_cb(UAV_msg):
    global UAV_pose
    UAV_pose = UAV_msg

##################
# Main function
##################

def main():

    rospy.init_node('gaussianPlume')
    rate = rospy.Rate(5)

    # Plume parameters
    QPlume        = rospy.get_param("/Q")                   #  kg/s     release rate
    vPlume        = rospy.get_param("/vPlume")              #  m/s      velocity
    DyPlume       = rospy.get_param("/DyPlume")             #  m        diffusion along y
    DzPlume       = rospy.get_param("/DzPlume")             #  m        diffusion along z

    UAVofIntrest  = rospy.get_param("gaussianPlumeSensor/UAVofIntrest")

    # Plume orientation translation then rotation
    xPlume     = rospy.get_param("/xPlume")       #  m
    yPlume     = rospy.get_param("/yPlume")       #  m
    zPlume     = rospy.get_param("/zPlume")       #  m
    thetaPlume = rospy.get_param("/thetaPlume")   #  degrees

    rospy.Subscriber("/" + UAVofIntrest  + "/" + "true_position", PoseStamped, UAV_cb)
    pub = rospy.Publisher("gaussianReading", gaussian, queue_size=10)

    thetaPlume = thetaPlume * (pi/180) # convert to radians

    while not rospy.is_shutdown():
        # Get reading

        # ppm
        tmpMsg.ppm = getReading(UAV_pose.pose.position.x, UAV_pose.pose.position.y, thetaPlume, xPlume, yPlume, UAV_pose.pose.position.z - zPlume, QPlume, vPlume, DyPlume, DzPlume)

        # Publish message
        pub.publish(tmpMsg);

        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
