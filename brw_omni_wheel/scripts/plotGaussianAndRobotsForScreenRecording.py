#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles
from quadnodes.msg import gaussian

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos, sqrt, exp

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

plotLists = [["111"], ["121", "122"], ["221", "222", "223"], ["221", "222", "223", "224"], ["231", "232", "233", "234", "235"]]

global Robot1_pose
global Robot2_pose
global Robot3_pose
global Robot4_pose
global Robot5_pose

global particlesCent
global gaussianMsg

Robot1_pose = PoseStamped()
Robot2_pose = PoseStamped()
Robot3_pose = PoseStamped()
Robot4_pose = PoseStamped()
Robot5_pose = PoseStamped()
plume_pose  = PoseStamped()

estimatedGaussianCent = estimatedGaussian()

particlesCent = particles()

##################
# Callbacks
##################

def Robot1_pose_cb(pose_cb_msg):
    global Robot1_pose
    Robot1_pose = pose_cb_msg

def Robot2_pose_cb(pose_cb_msg):
    global Robot2_pose
    Robot2_pose = pose_cb_msg

def Robot3_pose_cb(pose_cb_msg):
    global Robot3_pose
    Robot3_pose = pose_cb_msg

def Robot4_pose_cb(pose_cb_msg):
    global Robot4_pose
    Robot4_pose = pose_cb_msg

def Robot5_pose_cb(pose_cb_msg):
    global Robot5_pose
    Robot5_pose = pose_cb_msg


def gaussian_cb(gaussian_cb_msg):
    global estimatedGaussianCent
    estimatedGaussianCent = gaussian_cb_msg


##################
# Main function
##################

def main():
    rospy.init_node('plotGuassian')
    rate = rospy.Rate(10)

    # Map parameters
    minLimX = 0
    minLimY = 0
    maxLimX = 4.2
    maxLimY = 2.67

    maxPPMplot = 15

    plumeType       = rospy.get_param("/plumeType")      #  m        diffusion along z

    if plumeType == 1:
        # Plume parameters
        QPlume        = rospy.get_param("/QPlume")            #  kg/s     release rate
        vPlume        = rospy.get_param("/vPlume")       #  m/s      velocity
        DyPlume       = rospy.get_param("/DyPlume")      #  m        diffusion along y
        DzPlume       = rospy.get_param("/DzPlume")      #  m        diffusion along z

        # Plume orientation translation then rotation
        xPlume        = rospy.get_param("/xPlume")       #  m
        yPlume        = rospy.get_param("/yPlume")       #  m
        thetaPlume    = rospy.get_param("/thetaPlume")   #  degrees

    SpawnUAV1     = rospy.get_param("/subToR1Pose")
    SpawnUAV2     = rospy.get_param("/subToR2Pose")
    SpawnUAV3     = rospy.get_param("/subToR3Pose")
    SpawnUAV4     = rospy.get_param("/subToR4Pose")
    SpawnUAV5     = rospy.get_param("/subToR5Pose")

    robotList = []

    # Set up subscriptions
    if SpawnUAV1:
        rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, Robot1_pose_cb)
        xPltRobot1 = []
        yPltRobot1 = []
        robotList.append(1)

    if SpawnUAV2:
        rospy.Subscriber("/mocap_node/Robot_2/pose", PoseStamped, Robot2_pose_cb)
        xPltRobot2 = []
        yPltRobot2 = []
        robotList.append(2)

    if SpawnUAV3:
        rospy.Subscriber("/mocap_node/Robot_3/pose", PoseStamped, Robot3_pose_cb)
        xPltRobot3 = []
        yPltRobot3 = []
        robotList.append(3)

    if SpawnUAV4:
        rospy.Subscriber("/mocap_node/Robot_4/pose", PoseStamped, Robot4_pose_cb)
        xPltRobot4 = []
        yPltRobot4 = []
        robotList.append(4)

    if SpawnUAV5:
        rospy.Subscriber("/mocap_node/Robot_5/pose", PoseStamped, Robot5_pose_cb)
        xPltRobot5 = []
        yPltRobot5 = []
        robotList.append(5)

    robotAmount = len(robotList)

    xPlumePlot = np.arange(minLimX, maxLimX+1, 0.01)
    yPlumePlot = np.arange(minLimY, maxLimY+1, 0.01)
    conArrayTrue = np.zeros([len(yPlumePlot), len(xPlumePlot)])

    if plumeType == 1:
        for xCurrentIndex in range(len(xPlumePlot)):
            for yCurrentIndex in range(len(yPlumePlot)):
                conArrayTrue[yCurrentIndex,xCurrentIndex] = getReading(xPlumePlot[xCurrentIndex], yPlumePlot[yCurrentIndex], thetaPlume, xPlume, yPlume, 0, QPlume, vPlume, DyPlume, DzPlume)

        conArrayTrue[conArrayTrue > maxPPMplot] = maxPPMplot

    fig, ax = plt.subplots()
    fig.set_size_inches(8, 4.5, forward=True)
    while not rospy.is_shutdown():
        if SpawnUAV1:
            Robot1_poseXft = Robot1_pose.pose.position.x
            Robot1_poseYft = Robot1_pose.pose.position.y
            if Robot1_poseXft != 0.0 and Robot1_poseYft != 0.0:
                xPltRobot1.append(Robot1_poseXft)
                yPltRobot1.append(Robot1_poseYft)

        if SpawnUAV2:
            Robot2_poseXft = Robot2_pose.pose.position.x
            Robot2_poseYft = Robot2_pose.pose.position.y
            if Robot2_poseXft != 0.0 and Robot2_poseYft != 0.0:
                xPltRobot2.append(Robot2_poseXft)
                yPltRobot2.append(Robot2_poseYft)

        if SpawnUAV3:
            Robot3_poseXft = Robot3_pose.pose.position.x
            Robot3_poseYft = Robot3_pose.pose.position.y
            if Robot3_poseXft != 0.0 and Robot3_poseYft != 0.0:
                xPltRobot3.append(Robot3_poseXft)
                yPltRobot3.append(Robot3_poseYft)

        if SpawnUAV4:
            Robot4_poseXft = Robot4_pose.pose.position.x
            Robot4_poseYft = Robot4_pose.pose.position.y
            if Robot4_poseXft != 0.0 and Robot4_poseYft != 0.0:
                xPltRobot4.append(Robot4_poseXft)
                yPltRobot4.append(Robot4_poseYft)

        if SpawnUAV5:
            Robot5_poseXft = Robot5_pose.pose.position.x
            Robot5_poseYft = Robot5_pose.pose.position.y
            if Robot5_poseXft != 0.0 and Robot5_poseYft != 0.0:
                xPltRobot5.append(Robot5_poseXft)
                yPltRobot5.append(Robot5_poseYft)

        plt.clf()

        # Plotting stuff
        plt.subplot("111")

        # Plot gaussian
        if plumeType == 1:
            plt.contour(xPlumePlot,yPlumePlot,conArrayTrue,15)

        if SpawnUAV1 == 1:
            plt.plot(xPltRobot1, yPltRobot1, color='lime')
            plt.plot(Robot1_poseXft, Robot1_poseYft, 'D', markersize=10, color='lime')

        if SpawnUAV2 == 1:
            plt.plot(xPltRobot2, yPltRobot2, color='hotpink')
            plt.plot(Robot2_poseXft, Robot2_poseYft, 'D', markersize=10, color='hotpink')

        if SpawnUAV3 == 1:
            plt.plot(xPltRobot3, yPltRobot3, color='cyan')
            plt.plot(Robot3_poseXft, Robot3_poseYft, 'D', markersize=10, color='cyan')

        if SpawnUAV4 == 1:
            plt.plot(xPltRobot4, yPltRobot4, color='red')
            plt.plot(Robot4_poseXft, Robot4_poseYft, 'D', markersize=10, color='red')

        if SpawnUAV5 == 1:
            plt.plot(xPltRobot5, yPltRobot5, color='crimson')
            plt.plot(Robot5_poseXft, Robot5_poseYft, 'D', markersize=10, color='crimson')

        plt.axis("tight")  # gets rid of white border
        plt.margins(x=0)
        plt.tight_layout()
        plt.xlim(minLimX, maxLimX)
        plt.ylim(minLimY, maxLimY)
        plt.grid(True)
        plt.pause(0.01)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
