#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos, sqrt, exp
from geometry_msgs.msg import PoseStamped
from quadnodes.msg import gaussian
from mps_driver.msg import MPS
import playsound

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

global Robot1_pose
global Robot2_pose
global Robot3_pose
global Robot4_pose
global Robot5_pose

global Robot1_ppm
global Robot2_ppm
global Robot3_ppm
global Robot4_ppm
global Robot5_ppm

Robot1_pose = PoseStamped()
Robot2_pose = PoseStamped()
Robot3_pose = PoseStamped()
Robot4_pose = PoseStamped()
Robot5_pose = PoseStamped()

Robot1_ppm = 0
Robot2_ppm = 0
Robot3_ppm = 0
Robot4_ppm = 0
Robot5_ppm = 0

##################
# Callbacks
##################

def robot_1_pose_cb(pose_cb_msg):
    global Robot1_pose
    Robot1_pose = pose_cb_msg

def robot_2_pose_cb(pose_cb_msg):
    global Robot2_pose
    Robot2_pose = pose_cb_msg

def robot_3_pose_cb(pose_cb_msg):
    global Robot3_pose
    Robot3_pose = pose_cb_msg

def robot_4_pose_cb(pose_cb_msg):
    global Robot4_pose
    Robot4_pose = pose_cb_msg

def robot_5_pose_cb(pose_cb_msg):
    global Robot5_pose
    Robot5_pose = pose_cb_msg

def robot_1_gaussian_cb(gaussian_cb_msg):
    global Robot1_ppm
    Robot1_ppm = gaussian_cb_msg.ppm

def robot_2_gaussian_cb(gaussian_cb_msg):
    global Robot2_ppm
    Robot2_ppm = gaussian_cb_msg.ppm

def robot_3_gaussian_cb(gaussian_cb_msg):
    global Robot3_ppm
    Robot3_ppm = gaussian_cb_msg.ppm

def robot_4_gaussian_cb(gaussian_cb_msg):
    global Robot4_ppm
    Robot4_ppm = gaussian_cb_msg.ppm

def robot_5_gaussian_cb(gaussian_cb_msg):
    global Robot5_ppm
    Robot5_ppm = gaussian_cb_msg.ppm

def robot_1_MPS_cb(MPS_cb_msg):
    global Robot1_ppm
    Robot1_ppm = MPS_cb_msg.pressure

def robot_2_MPS_cb(MPS_cb_msg):
    global Robot2_ppm
    Robot2_ppm = MPS_cb_msg.pressure

def robot_3_MPS_cb(MPS_cb_msg):
    global Robot3_ppm
    Robot3_ppm = MPS_cb_msg.pressure

def robot_4_MPS_cb(MPS_cb_msg):
    global Robot4_ppm
    Robot4_ppm = MPS_cb_msg.pressure

def robot_5_MPS_cb(MPS_cb_msg):
    global Robot5_ppm
    Robot5_ppm = MPS_cb_msg.pressure


##################
# Main function
##################

def main():

    if np.random.rand() < 0.01:
        # mixer.init()
        # mixer.music.load('~/mocaplinux/catkin_ws/src/NNvisit/scripts/okayThis.mp3')
        # mixer.music.play()

        # p = vlc.MediaPlayer('~/mocaplinux/catkin_ws/src/NNvisit/scripts/okayThis.mp3')
        # p.play()

        playsound.playsound('okayThis.mp3')

    rospy.init_node('plotGuassian')
    rate = rospy.Rate(30)


    # Map parameters
    minLimX = 0
    minLimY = 0
    maxLimX = 14 / 3.28084
    maxLimY = 9 / 3.28084

    maxPPMplot = 15

    plumeType        = rospy.get_param("/plumeType")            #  kg/s     release rate

    if plumeType == 1:
        # Plume parameters
        QPlume        = rospy.get_param("/QPlume")            #  kg/s     release rate
        vPlume        = rospy.get_param("/vPlume")       #  m/s      velocity
        DyPlume       = rospy.get_param("/DyPlume")      #  m        diffusion along y
        DzPlume       = rospy.get_param("/DzPlume")      #  m        diffusion along z

        # Plume orientation translation then rotation
        xPlume     = rospy.get_param("/xPlume")       #  m
        yPlume     = rospy.get_param("/yPlume")       #  m
        thetaPlume = rospy.get_param("/thetaPlume")   #  degrees

        rospy.Subscriber("/Robot_1/gaussianReading", gaussian, robot_1_gaussian_cb)
        rospy.Subscriber("/Robot_2/gaussianReading", gaussian, robot_2_gaussian_cb)
        rospy.Subscriber("/Robot_3/gaussianReading", gaussian, robot_3_gaussian_cb)
        rospy.Subscriber("/Robot_4/gaussianReading", gaussian, robot_4_gaussian_cb)
        rospy.Subscriber("/Robot_5/gaussianReading", gaussian, robot_5_gaussian_cb)

    elif plumeType == 2:
        pass

    elif plumeType == 3:
        rospy.Subscriber("/Robot_1/mps_data", MPS, robot_1_MPS_cb)
        rospy.Subscriber("/Robot_2/mps_data", MPS, robot_2_MPS_cb)
        rospy.Subscriber("/Robot_3/mps_data", MPS, robot_3_MPS_cb)
        rospy.Subscriber("/Robot_4/mps_data", MPS, robot_4_MPS_cb)
        rospy.Subscriber("/Robot_5/mps_data", MPS, robot_5_MPS_cb)

    # xPlume = xPlume * 3.28084
    # yPlume = yPlume * 3.28084

    # Set up subscriptions
    rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, robot_1_pose_cb)
    rospy.Subscriber("/mocap_node/Robot_2/pose", PoseStamped, robot_2_pose_cb)
    rospy.Subscriber("/mocap_node/Robot_3/pose", PoseStamped, robot_3_pose_cb)
    rospy.Subscriber("/mocap_node/Robot_4/pose", PoseStamped, robot_4_pose_cb)
    rospy.Subscriber("/mocap_node/Robot_5/pose", PoseStamped, robot_5_pose_cb)

    if plumeType == 1:
        thetaPlume = thetaPlume * (pi/180) # convert to radians
        xPlumePlot = np.arange(minLimX, maxLimX+1, 0.01)
        yPlumePlot = np.arange(minLimY, maxLimY+1, 0.01)

        conArray = np.zeros([len(yPlumePlot), len(xPlumePlot)])

        for xCurrentIndex in range(len(xPlumePlot)):
            for yCurrentIndex in range(len(yPlumePlot)):
                conArray[yCurrentIndex,xCurrentIndex] = getReading(xPlumePlot[xCurrentIndex], yPlumePlot[yCurrentIndex], thetaPlume, xPlume, yPlume, 0.0, QPlume, vPlume, DyPlume, DzPlume)

        conArray[conArray > maxPPMplot] = maxPPMplot

    while not rospy.is_shutdown():
        # Plotting stuff
        plt.subplot('111')
        plt.clf()

        # Plot gaussian
        if plumeType == 1:
            plt.contour(xPlumePlot,yPlumePlot,conArray,15)


        plt.plot(Robot1_pose.pose.position.x, Robot1_pose.pose.position.y,'ro',  markersize=6)
        plt.plot(Robot2_pose.pose.position.x, Robot2_pose.pose.position.y,'ro',  markersize=6)
        plt.plot(Robot3_pose.pose.position.x, Robot3_pose.pose.position.y,'ro',  markersize=6)
        plt.plot(Robot4_pose.pose.position.x, Robot4_pose.pose.position.y,'ro',  markersize=6)
        plt.plot(Robot5_pose.pose.position.x, Robot5_pose.pose.position.y,'ro',  markersize=6)

        # print(Robot4_ppm)

        plt.text(Robot1_pose.pose.position.x + 0.025, Robot1_pose.pose.position.y - 0.1, "R1:" + str(round(Robot1_ppm, 3)))
        plt.text(Robot2_pose.pose.position.x + 0.025, Robot2_pose.pose.position.y - 0.1, "R2:" + str(round(Robot2_ppm, 3)))
        plt.text(Robot3_pose.pose.position.x + 0.025, Robot3_pose.pose.position.y - 0.1, "R3:" + str(round(Robot3_ppm, 3)))
        plt.text(Robot4_pose.pose.position.x + 0.025, Robot4_pose.pose.position.y - 0.1, "R4:" + str(round(Robot4_ppm, 3)))
        plt.text(Robot5_pose.pose.position.x + 0.025, Robot5_pose.pose.position.y - 0.1, "R5:" + str(round(Robot5_ppm, 3)))

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
