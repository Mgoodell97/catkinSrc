#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from geometry_msgs.msg import PoseStamped
from quadnodes.msg import gaussian

from math import cos, sin, pi, acos, sqrt, exp
import numpy as np

##################
# Global variables
##################

global Robot1_pose
Robot1_pose = PoseStamped()

tmpMsg = gaussian()
# global pose_cb_flag
# pose_cb_flag = True

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
# Callbacks
##################

def Robot1_pose_cb(Robot1_pose_msg):
    global Robot1_pose
    Robot1_pose = Robot1_pose_msg

##################
# Main
##################

def main():
    rospy.init_node('gaussianPlumeSensor')
    rate = rospy.Rate(50)

    # Plume parameters
    QPlume        = rospy.get_param("/QPlume")                   #  kg/s     release rate
    vPlume        = rospy.get_param("/vPlume")              #  m/s      velocity
    DyPlume       = rospy.get_param("/DyPlume")             #  m        diffusion along y
    DzPlume       = rospy.get_param("/DzPlume")             #  m        diffusion along z

    # Plume orientation translation then rotation
    xPlume     = rospy.get_param("/xPlume")       #  m
    yPlume     = rospy.get_param("/yPlume")       #  m
    zPlume     = rospy.get_param("/zPlume")       #  m
    thetaPlume = rospy.get_param("/thetaPlume")   #  degrees
    careAboutZ = rospy.get_param("/careAboutZ")   #  bool
    sensorRate = rospy.get_param("/sensorRate")   #  Hz

    RobotID    = rospy.get_param("gaussianSensorPlatform/RobotID")   #  Hz

    fullStringName = "/mocap_node/Robot_" + str(int(RobotID)) + "/pose"

    rospy.Subscriber(fullStringName, PoseStamped, Robot1_pose_cb)
    pub = rospy.Publisher("gaussianReading", gaussian, queue_size=10)

    thetaPlume = thetaPlume * (pi/180) # convert to radians

    rate = rospy.Rate(sensorRate)

    while not rospy.is_shutdown():
        # Get reading

        # ppm
        if careAboutZ:
            tmpMsg.ppm = getReading(Robot1_pose.pose.position.x, Robot1_pose.pose.position.y, thetaPlume, xPlume, yPlume, Robot1_pose.pose.position.z - zPlume, QPlume, vPlume, DyPlume, DzPlume)

        else:
            tmpMsg.ppm = getReading(Robot1_pose.pose.position.x, Robot1_pose.pose.position.y, thetaPlume, xPlume, yPlume, 0, QPlume, vPlume, DyPlume, DzPlume)
           #  tmpMsg.ppm = getReading(Robot1_pose.pose.position.x, Robot1_pose.pose.position.y, thetaPlume, xPlume, yPlume, 0, QPlume, vPlume, DyPlume, DzPlume) + np.random.normal(0, 1)

        tmpMsg.x = Robot1_pose.pose.position.x
        tmpMsg.y = Robot1_pose.pose.position.y
        tmpMsg.z = Robot1_pose.pose.position.z

        # Publish message
        pub.publish(tmpMsg);

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
