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
    return con

def getReading(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    # Rotate frame

    Xw_r = np.array([xRobotDef, yRobotDef, 1])

    R = np.array([[cos(thetaFunc), -sin(thetaFunc)], [sin(thetaFunc), cos(thetaFunc)]])
    P = np.array([[xPlumeFunc, yPlumeFunc]]).T;

    bottomArray = np.array([[0, 0 , 1]]);
    # Tw_plumeFrame = np.concatenate((R, P), axis=1)

    # # Transfer matrix from plumeframe to w
    # Tw_plumeFrame = np.concatenate((Tw_plumeFrame, bottomArray), axis=0)

    Rinv = np.linalg.inv(R)
    Pinv = np.array([np.matmul(-Rinv, np.squeeze(P))]).T

    TplumeFrame_w = np.concatenate((Rinv, Pinv), axis=1)

    # Transfer matrix from w to plumeframe
    TplumeFrame_w = np.concatenate((TplumeFrame_w, bottomArray), axis=0)

    XplumeFrame = np.matmul(TplumeFrame_w, Xw_r)

    xRobotRotated = XplumeFrame[0]
    yRobotRotated = XplumeFrame[1]

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
    PPM_at_center = rospy.get_param("/PPM_at_center")       #  kg/s     release rate
    vPlume        = rospy.get_param("/vPlume")              #  m/s      velocity
    DyPlume       = rospy.get_param("/DyPlume")             #  m        diffusion along y
    DzPlume       = rospy.get_param("/DzPlume")             #  m        diffusion along z
    DiameterPlume = rospy.get_param("/DiameterPlume")       #  m        diameter of release valve

    UAVofIntrest  = rospy.get_param("gaussianPlumeSensor/UAVofIntrest")

    releaseArea   = pi * pow((DiameterPlume/2),2)
    QPlume = releaseArea * vPlume * PPM_at_center/1000

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
        # kg/s
        tmpMsg.kg_s = getReading(UAV_pose.pose.position.x, UAV_pose.pose.position.y, thetaPlume, xPlume, yPlume, zPlume - UAV_pose.pose.position.z, QPlume, vPlume, DyPlume, DzPlume)

        # ppm
        tmpMsg.ppm = tmpMsg.kg_s * (1/vPlume) * (1/releaseArea) * 1000

        # Publish message
        pub.publish(tmpMsg);

        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
