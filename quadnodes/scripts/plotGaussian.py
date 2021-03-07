#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos, sqrt, exp
from geometry_msgs.msg import PoseStamped

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

global UAV1_pose
global UAV2_pose
global UAV3_pose
UAV1_pose = PoseStamped()
UAV2_pose = PoseStamped()
UAV3_pose = PoseStamped()

##################
# Callbacks
##################

def UAV1_cb(UAV1_msg):
    global UAV1_pose
    UAV1_pose = UAV1_msg

def UAV2_cb(UAV2_msg):
    global UAV2_pose
    UAV2_pose = UAV2_msg

def UAV3_cb(UAV3_msg):
    global UAV3_pose
    UAV3_pose = UAV3_msg

##################
# Main function
##################

def main():

    rospy.init_node('plotGuassian')
    rate = rospy.Rate(5)

    plotUAV1 = rospy.get_param("/plotGaussian/plotUAV1")
    plotUAV2 = rospy.get_param("/plotGaussian/plotUAV2")
    plotUAV3 = rospy.get_param("/plotGaussian/plotUAV3")

    # Map parameters
    minLim = rospy.get_param("/plotGaussian/mapMin")
    maxLim = rospy.get_param("/plotGaussian/mapMax")

    # Plume parameters
    PPM_at_center = rospy.get_param("/PPM_at_center")       #  kg/s     release rate
    vPlume        = rospy.get_param("/vPlume")       #  m/s      velocity
    DyPlume       = rospy.get_param("/DyPlume")      #  m        diffusion along y
    DzPlume       = rospy.get_param("/DzPlume")      #  m        diffusion along z
    DiameterPlume = rospy.get_param("/DiameterPlume")       #  m        diameter of release valve
    releaseArea   = pi * pow((DiameterPlume/2),2)
    QPlume = releaseArea * vPlume * PPM_at_center/1000

    # Plume orientation translation then rotation
    xPlume     = rospy.get_param("/xPlume")       #  m
    yPlume     = rospy.get_param("/yPlume")       #  m
    thetaPlume = rospy.get_param("/thetaPlume")   #  degrees

    if plotUAV1:
        rospy.Subscriber("UAV1/true_position", PoseStamped, UAV1_cb)
        xPltUAV1 = []
        yPltUAV1 = []
    if plotUAV2:
        rospy.Subscriber("UAV2/true_position", PoseStamped, UAV2_cb)
        xPltUAV2 = []
        yPltUAV2 = []
    if plotUAV3:
        rospy.Subscriber("UAV3/true_position", PoseStamped, UAV3_cb)
        xPltUAV3 = []
        yPltUAV3 = []



    thetaPlume = thetaPlume * (pi/180) # convert to radians
    xPlumePlot = np.arange(minLim, maxLim+1, 0.5)
    yPlumePlot = np.arange(minLim, maxLim+1, 0.5)

    conArray = np.zeros([len(xPlumePlot), len(yPlumePlot)])

    for xCurrentIndex in range(len(xPlumePlot)):
        for yCurrentIndex in range(len(yPlumePlot)):
            conArray[xCurrentIndex,yCurrentIndex] = getReading(yPlumePlot[yCurrentIndex], xPlumePlot[xCurrentIndex], thetaPlume, xPlume, yPlume, 0.0, QPlume, vPlume, DyPlume, DzPlume)

    flat=conArray.flatten()
    flat.sort()
    secondHighestValue = flat[-2]
    ind = np.unravel_index(np.argmax(conArray, axis=None), conArray.shape)
    conArray[ind[0]][ind[1]] = secondHighestValue * 1.03

    while not rospy.is_shutdown():
        # Plotting stuff
        plt.clf()

        # Plot gaussian
        plt.contourf(xPlumePlot,yPlumePlot,conArray,25)

        # Plot UAV1
        if plotUAV1:
            if UAV1_pose.pose.position.x != 0.0:
                xPltUAV1.append(UAV1_pose.pose.position.x)
                yPltUAV1.append(UAV1_pose.pose.position.y)
            plt.plot(xPltUAV1, yPltUAV1,"r")
            plt.plot(UAV1_pose.pose.position.x,UAV1_pose.pose.position.y,'ro',  markersize=6)

        # Plot UAV2
        if plotUAV2:
            if UAV2_pose.pose.position.x != 0.0:
                xPltUAV2.append(UAV2_pose.pose.position.x)
                yPltUAV2.append(UAV2_pose.pose.position.y)
            plt.plot(xPltUAV2, yPltUAV2,"g")
            plt.plot(UAV2_pose.pose.position.x,UAV2_pose.pose.position.y,'go', markersize=6)

        # Plot UAV3
        if plotUAV3:
            if UAV3_pose.pose.position.x != 0.0:
                xPltUAV3.append(UAV3_pose.pose.position.x)
                yPltUAV3.append(UAV3_pose.pose.position.y)
            plt.plot(xPltUAV3, yPltUAV3,'b')
            plt.plot(UAV3_pose.pose.position.x,UAV3_pose.pose.position.y,'bo', markersize=6)

        plt.xlim(minLim, maxLim)
        plt.ylim(minLim, maxLim)
        rate.sleep()
        plt.pause(0.05)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
