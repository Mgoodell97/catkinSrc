#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles
from quadnodes.msg import gaussian
from particle_filter.msg import estimatedGaussian

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

global Robot1_particles
global Robot2_particles
global Robot3_particles
global Robot4_particles
global Robot5_particles

global Robot1_EstimatedGaussian
global Robot2_EstimatedGaussian
global Robot3_EstimatedGaussian
global Robot4_EstimatedGaussian
global Robot5_EstimatedGaussian

Robot1_pose = PoseStamped()
Robot2_pose = PoseStamped()
Robot3_pose = PoseStamped()
Robot4_pose = PoseStamped()
Robot5_pose = PoseStamped()

Robot1_particles = particles()
Robot2_particles = particles()
Robot3_particles = particles()
Robot4_particles = particles()
Robot5_particles = particles()

Robot1_EstimatedGaussian = estimatedGaussian()
Robot2_EstimatedGaussian = estimatedGaussian()
Robot3_EstimatedGaussian = estimatedGaussian()
Robot4_EstimatedGaussian = estimatedGaussian()
Robot5_EstimatedGaussian = estimatedGaussian()

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



def Robot1_particles_cb(particles_msg):
    global Robot1_particles
    Robot1_particles = particles_msg

def Robot2_particles_cb(particles_msg):
    global Robot2_particles
    Robot2_particles = particles_msg

def Robot3_particles_cb(particles_msg):
    global Robot3_particles
    Robot3_particles = particles_msg

def Robot4_particles_cb(particles_msg):
    global Robot4_particles
    Robot4_particles = particles_msg

def Robot5_particles_cb(particles_msg):
    global Robot5_particles
    Robot5_particles = particles_msg


def Robot1_estimatedGaussian_cb(estimatedGaussian_cb_msg):
    global Robot1_EstimatedGaussian
    Robot1_EstimatedGaussian = estimatedGaussian_cb_msg



##################
# Main function
##################

def main():
    rospy.init_node('plotGuassian')
    rate = rospy.Rate(30)

    # Map parameters
    minLimX = 0
    minLimY = 0
    maxLimX = 14 / 3.28084
    maxLimY = 9 / 3.28084

    maxPPMplot = 15

    SpawnUAV1     = rospy.get_param("/SpawnUAV1")
    SpawnUAV2     = rospy.get_param("/SpawnUAV2")
    SpawnUAV3     = rospy.get_param("/SpawnUAV3")
    SpawnUAV4     = rospy.get_param("/SpawnUAV4")
    SpawnUAV5     = rospy.get_param("/SpawnUAV5")

    robotList = []

    # Set up subscriptions
    if SpawnUAV1:
        rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, Robot1_pose_cb)
        rospy.Subscriber("Robot_1/particles", particles, Robot1_particles_cb)
        rospy.Subscriber("Robot_1/estimatedGaussian", estimatedGaussian, Robot1_estimatedGaussian_cb)
        xPltRobot1 = []
        yPltRobot1 = []
        robotList.append(1)

    if SpawnUAV2:
        rospy.Subscriber("/mocap_node/Robot_2/pose", PoseStamped, Robot2_pose_cb)
        rospy.Subscriber("Robot_2/particles", particles, Robot2_particles_cb)
        xPltRobot2 = []
        yPltRobot2 = []
        robotList.append(2)

    if SpawnUAV3:
        rospy.Subscriber("/mocap_node/Robot_3/pose", PoseStamped, Robot3_pose_cb)
        rospy.Subscriber("Robot_3/particles", particles, Robot3_particles_cb)
        xPltRobot3 = []
        yPltRobot3 = []
        robotList.append(3)

    if SpawnUAV4:
        rospy.Subscriber("/mocap_node/Robot_4/pose", PoseStamped, Robot4_pose_cb)
        rospy.Subscriber("Robot_4/particles", particles, Robot4_particles_cb)
        xPltRobot4 = []
        yPltRobot4 = []
        robotList.append(4)

    if SpawnUAV5:
        rospy.Subscriber("/mocap_node/Robot_5/pose", PoseStamped, Robot5_pose_cb)
        rospy.Subscriber("Robot_5/particles", particles, Robot5_particles_cb)
        xPltRobot5 = []
        yPltRobot5 = []
        robotList.append(5)

    robotAmount = len(robotList)

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
        for robotIDindex, plotString  in enumerate(plotLists[robotAmount-1]):
           # plt.subplot(plotString)
           # Plot gaussian
           plt.figure(1)
           # plt.contour(xPlumePlot,yPlumePlot,conArray,25)

           if robotList[robotIDindex] == 1:
                plt.plot(xPltRobot1, yPltRobot1,"r")
                plt.plot(Robot1_poseXft, Robot1_poseYft,'rD',  markersize=6)
                plt.plot(Robot1_particles.X,Robot1_particles.Y,'b.')

           if robotList[robotIDindex] == 2:
                plt.plot(xPltRobot2, yPltRobot2,"r")
                plt.plot(Robot2_poseXft, Robot2_poseYft,'rD',  markersize=6)
                plt.plot(Robot2_particles.X,Robot2_particles.Y,'g.')

           if robotList[robotIDindex] == 3:
                plt.plot(xPltRobot3, yPltRobot3,"r")
                plt.plot(Robot3_poseXft, Robot3_poseYft,'rD',  markersize=6)
                plt.plot(Robot3_particles.X,Robot3_particles.Y,'m.')

           if robotList[robotIDindex] == 4:
                plt.plot(xPltRobot4, yPltRobot4,"r")
                plt.plot(Robot4_poseXft, Robot4_poseYft,'rD',  markersize=6)
                plt.plot(Robot4_particles.X,Robot4_particles.Y,'c.')

           if robotList[robotIDindex] == 5:
                plt.plot(xPltRobot5, yPltRobot5,"r")
                plt.plot(Robot5_poseXft, Robot5_poseYft,'rD',  markersize=6)
                plt.plot(Robot5_particles.X,Robot5_particles.Y,'y.')

        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.grid()
        plt.xlim(minLimX, maxLimX)
        plt.ylim(minLimY, maxLimY)

        plt.pause(0.01)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
