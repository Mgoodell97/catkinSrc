#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos, sqrt, exp
from geometry_msgs.msg import PoseStamped
from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles
from matplotlib import cm

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
global UAV1_predict_gauss
global UAV2_predict_gauss
global UAV3_predict_gauss
UAV1_predict_gauss = estimatedGaussian()
UAV2_predict_gauss = estimatedGaussian()
UAV3_predict_gauss = estimatedGaussian()
global UAV1_particles
global UAV2_particles
global UAV3_particles
UAV1_particles = particles()
UAV2_particles = particles()
UAV3_particles = particles()

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

def UAV1_ep(UAV1_msg_ep):
    global UAV1_predict_gauss
    UAV1_predict_gauss= UAV1_msg_ep

def UAV2_ep(UAV2_msg_ep):
    global UAV2_predict_gauss
    UAV2_predict_gauss= UAV2_msg_ep

def UAV3_ep(UAV3_msg_ep):
    global UAV3_predict_gauss
    UAV3_predict_gauss= UAV3_msg_ep

def UAV1_particles_cb(UAV1_particles_msg):
    global UAV1_particles
    UAV1_particles = UAV1_particles_msg

def UAV2_particles_cb(UAV2_particles_msg):
    global UAV2_particles
    UAV2_particles = UAV2_particles_msg

def UAV3_particles_cb(UAV3_particles_msg):
    global UAV3_particles
    UAV3_particles = UAV3_particles_msg

##################
# Main function
##################

def main():
    rospy.init_node('plotGuassianwithPF')
    rate = rospy.Rate(20)

    try:
        plotUAV1 = rospy.get_param("/plotGaussianwithPF/plotUAV1")
    except:
        plotUAV1=False
    try:
        plotUAV2 = rospy.get_param("/plotGaussianwithPF/plotUAV2")
    except:
        plotUAV2 = False
    try:
        plotUAV3 = rospy.get_param("/plotGaussianwithPF/plotUAV3")
    except:
        plotUAV3 = False
    # Map parameters
    minLim = rospy.get_param("/plotGaussianwithPF/mapMin")
    maxLim = rospy.get_param("/plotGaussianwithPF/mapMax")

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
        rospy.Subscriber("UAV1/gaussianEstimation", estimatedGaussian, UAV1_ep)
        rospy.Subscriber("UAV1/particles", particles, UAV1_particles_cb)
        xPltUAV1 = []
        yPltUAV1 = []
    if plotUAV2:
        rospy.Subscriber("UAV2/true_position", PoseStamped, UAV2_cb)
        rospy.Subscriber("UAV2/gaussianEstimation", estimatedGaussian, UAV2_ep)
        rospy.Subscriber("UAV2/particles", particles, UAV2_particles_cb)
        xPltUAV2 = []
        yPltUAV2 = []

    if plotUAV3:
        rospy.Subscriber("UAV3/true_position", PoseStamped, UAV3_cb)
        rospy.Subscriber("UAV3/gaussianEstimation", estimatedGaussian, UAV3_ep)
        rospy.Subscriber("UAV3/particles", particles, UAV3_particles_cb)
        xPltUAV3 = []
        yPltUAV3 = []

    thetaPlume = thetaPlume * (pi/180) # convert to radians
    xPlumePlot = np.arange(minLim, maxLim+1, 0.5)
    yPlumePlot = np.arange(minLim, maxLim+1, 0.5)

    conArray = np.zeros([len(xPlumePlot), len(yPlumePlot)])
    conArray_estimated = conArray.copy()

    for xCurrentIndex in range(len(xPlumePlot)):
        for yCurrentIndex in range(len(yPlumePlot)):
            conArray[xCurrentIndex,yCurrentIndex] = getReading(yPlumePlot[yCurrentIndex], xPlumePlot[xCurrentIndex], thetaPlume, xPlume, yPlume, 0.0, QPlume, vPlume, DyPlume, DzPlume)* (1/vPlume) * (1/releaseArea) * 1000


    flat=conArray.flatten()
    flat.sort()

    secondHighestValue = flat[-2]
    ind = np.unravel_index(np.argmax(conArray, axis=None), conArray.shape)
    conArray[ind[0]][ind[1]] = secondHighestValue * 1.03



    while not rospy.is_shutdown():
        # Plotting stuff
        plt.clf()


        if plotUAV3:
            xPlume_estimated = (UAV1_predict_gauss.X+UAV2_predict_gauss.X+UAV3_predict_gauss.X)/3
            yPlume_estimated = (UAV1_predict_gauss.Y+UAV2_predict_gauss.Y+UAV3_predict_gauss.Y)/3
            zPlume_estimated = (UAV1_predict_gauss.Z+UAV2_predict_gauss.Z+UAV3_predict_gauss.Z)/3
            ThetaPlume_estimated =(UAV1_predict_gauss.Theta+UAV2_predict_gauss.Theta+UAV3_predict_gauss.Theta)/3
            QPlume_estimated = (UAV1_predict_gauss.Q+UAV2_predict_gauss.Q+UAV3_predict_gauss.Q)/3
            VPlume_estimated = (UAV1_predict_gauss.V+UAV2_predict_gauss.V+UAV3_predict_gauss.V)/3
            DyPlume_estimated = (UAV1_predict_gauss.Dy+UAV2_predict_gauss.Dy+UAV3_predict_gauss.Dy)/3
            DzPlume_estimated = (UAV1_predict_gauss.Dz+UAV2_predict_gauss.Dz+UAV3_predict_gauss.Dz)/3
            plt.subplot(222)
            plt.plot(UAV1_particles.X[0:5000],UAV1_particles.Y[0:5000],'r.')
            plt.xlabel("x [m]")
            plt.ylabel("y [m]")
            plt.grid()
            plt.xlim(minLim, maxLim)
            plt.ylim(minLim, maxLim)
            plt.subplot(223)
            plt.plot(UAV2_particles.X[0:5000],UAV2_particles.Y[0:5000],'g.')
            plt.xlabel("x [m]")
            plt.ylabel("y [m]")
            plt.grid()
            plt.xlim(minLim, maxLim)
            plt.ylim(minLim, maxLim)
            plt.subplot(224)
            plt.plot(UAV3_particles.X[0:5000],UAV3_particles.Y[0:5000],'b.')
            plt.xlabel("x [m]")
            plt.ylabel("y [m]")
            plt.grid()
            plt.xlim(minLim, maxLim)
            plt.ylim(minLim, maxLim)
            plumeSubplot = "221"
        elif plotUAV2:
            xPlume_estimated = (UAV1_predict_gauss.X+UAV2_predict_gauss.X)/2
            yPlume_estimated = (UAV1_predict_gauss.Y+UAV2_predict_gauss.Y)/2
            zPlume_estimated = (UAV1_predict_gauss.Z+UAV2_predict_gauss.Z)/2
            ThetaPlume_estimated =(UAV1_predict_gauss.Theta+UAV2_predict_gauss.Theta)/2
            QPlume_estimated = (UAV1_predict_gauss.Q+UAV2_predict_gauss.Q)/2
            VPlume_estimated = (UAV1_predict_gauss.V+UAV2_predict_gauss.V)/2
            DyPlume_estimated = (UAV1_predict_gauss.Dy+UAV2_predict_gauss.Dy)/2
            DzPlume_estimated = (UAV1_predict_gauss.Dz+UAV2_predict_gauss.Dz)/2
            plt.subplot(132)
            plt.plot(UAV1_particles.X[0:5000],UAV1_particles.Y[0:5000],'r.')
            plt.xlabel("x [m]")
            plt.ylabel("y [m]")
            plt.grid()
            plt.xlim(minLim, maxLim)
            plt.ylim(minLim, maxLim)
            plt.subplot(133)
            plt.plot(UAV2_particles.X[0:5000],UAV2_particles.Y[0:5000],'g.')
            plt.xlabel("x [m]")
            plt.ylabel("y [m]")
            plt.grid()
            plt.xlim(minLim, maxLim)
            plt.ylim(minLim, maxLim)
            plumeSubplot = "131"
        elif plotUAV1:
            xPlume_estimated = UAV1_predict_gauss.X
            yPlume_estimated = UAV1_predict_gauss.Y
            zPlume_estimated = UAV1_predict_gauss.Z
            ThetaPlume_estimated =UAV1_predict_gauss.Theta
            QPlume_estimated = UAV1_predict_gauss.Q
            VPlume_estimated = UAV1_predict_gauss.V
            DyPlume_estimated = UAV1_predict_gauss.Dy
            DzPlume_estimated = UAV1_predict_gauss.Dz
            plt.subplot(122)
            plt.plot(UAV1_particles.X[0:5000],UAV1_particles.Y[0:5000],'r.')
            plt.xlabel("x [m]")
            plt.ylabel("y [m]")
            plt.grid()
            plt.xlim(minLim, maxLim)
            plt.ylim(minLim, maxLim)
            plumeSubplot = "121"


        for xCurrentIndex in range(len(xPlumePlot)):
            for yCurrentIndex in range(len(yPlumePlot)):
                if (DzPlume_estimated != 0) and (DyPlume_estimated !=0):
                    conArray_estimated[xCurrentIndex,yCurrentIndex] = getReading(yPlumePlot[yCurrentIndex], xPlumePlot[xCurrentIndex], ThetaPlume_estimated, xPlume_estimated, yPlume_estimated, 0, QPlume_estimated, VPlume_estimated, DyPlume_estimated, DzPlume_estimated)

        # Plot gaussian
        plt.subplot(plumeSubplot)
        plt.contourf(xPlumePlot,yPlumePlot,conArray,25,cmap=cm.jet)
        #plt.figure(1)
        plt.contour(xPlumePlot,yPlumePlot,conArray_estimated,25,cmap=cm.jet)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")


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
        plt.pause(0.01)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
