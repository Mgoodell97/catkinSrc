#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos, sqrt, exp
from geometry_msgs.msg import PoseStamped
from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles
from std_msgs.msg import Float32MultiArray
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

global UAV_pose
global UAV_predict_gauss
global UAV_particles
UAV_pose = PoseStamped()
UAV_predict_gauss = estimatedGaussian()
UAV_particles = particles()

global particleMatrixMSG
particleMatrixMSG = Float32MultiArray()

##################
# Callbacks
##################

def UAV_cb(UAV_pose_msg):
    global UAV_pose
    UAV_pose = UAV_pose_msg

def UAV_ep(UAV_msg_ep):
    global UAV_predict_gauss
    UAV_predict_gauss= UAV_msg_ep

def UAV_particles_cb(UAV_particles_msg):
    global UAV_particles
    UAV_particles = UAV_particles_msg

def matrix_cb(particleMatrixMSG_cb):
    global particleMatrixMSG
    particleMatrixMSG = particleMatrixMSG_cb

##################
# Main function
##################

def main():
    rospy.init_node('plotGuassianwithPF')
    rate = rospy.Rate(10)

    # Map parameters
    rowSteps           = rospy.get_param("rowSteps")
    colSteps           = rospy.get_param("colSteps")
    xMinMap            = rospy.get_param("xMinMap")
    yMinMap            = rospy.get_param("yMinMap")
    xMaxMap            = rospy.get_param("xMaxMap")
    yMaxMap            = rospy.get_param("yMaxMap")
    particleColor      = rospy.get_param("graphSearchPlots/particleColor")

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


    rospy.Subscriber("true_position", PoseStamped, UAV_cb)
    rospy.Subscriber("gaussianEstimation", estimatedGaussian, UAV_ep)
    rospy.Subscriber("particles", particles, UAV_particles_cb)
    # rospy.Subscriber("patriclesArray", Float32MultiArray, matrix_cb)

    xPltUAV = []
    yPltUAV = []

    thetaPlume = thetaPlume * (pi/180) # convert to radians
    xPlumePlot = np.arange(xMinMap, xMaxMap+1, 0.5)
    yPlumePlot = np.arange(yMinMap, yMaxMap+1, 0.5)

    conArray = np.zeros([len(xPlumePlot), len(yPlumePlot)])
    conArray_estimated = conArray.copy()

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

        xPlume_estimated = UAV_predict_gauss.X
        yPlume_estimated = UAV_predict_gauss.Y
        zPlume_estimated = UAV_predict_gauss.Z
        ThetaPlume_estimated =UAV_predict_gauss.Theta
        QPlume_estimated = UAV_predict_gauss.Q
        VPlume_estimated = UAV_predict_gauss.V
        DyPlume_estimated = UAV_predict_gauss.Dy
        DzPlume_estimated = UAV_predict_gauss.Dz
        plt.subplot(131)
        if particleColor == "red":
            plt.plot(UAV_particles.X[0:5000],UAV_particles.Y[0:5000],'r.')
        elif particleColor == "green":
            plt.plot(UAV_particles.X[0:5000],UAV_particles.Y[0:5000],'g.')
        elif particleColor == "blue":
            plt.plot(UAV_particles.X[0:5000],UAV_particles.Y[0:5000],'b.')
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.grid()
        plt.xlim(xMinMap, xMaxMap)
        plt.ylim(yMinMap, yMaxMap)

        plt.subplot(132)
        maxBinCBar = 0
        xEdgesTemp = np.linspace(xMinMap, xMaxMap, num=colSteps+1)
        yEdgesTemp = np.linspace(yMinMap, yMaxMap, num=rowSteps+1)
        patriclesArray, xedgesPlot, yedgesPlot = np.histogram2d(UAV_particles.Y, UAV_particles.X, bins=(yEdgesTemp, xEdgesTemp))
        currentMax = patriclesArray.max()
        if maxBinCBar < currentMax:
            maxBinCBar = currentMax
        plt.imshow(patriclesArray, interpolation='nearest', origin='lower', extent=[yedgesPlot[0], yedgesPlot[-1],xedgesPlot[0], xedgesPlot[-1]], aspect="auto")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")


        for xCurrentIndex in range(len(xPlumePlot)):
            for yCurrentIndex in range(len(yPlumePlot)):
                if (DzPlume_estimated != 0) and (DyPlume_estimated !=0):
                    conArray_estimated[xCurrentIndex,yCurrentIndex] = getReading(yPlumePlot[yCurrentIndex], xPlumePlot[xCurrentIndex], ThetaPlume_estimated, xPlume_estimated, yPlume_estimated, 0.0, QPlume_estimated, VPlume_estimated, DyPlume_estimated, DzPlume_estimated)

        # Plot gaussian
        plt.subplot(133)
        plt.contourf(xPlumePlot,yPlumePlot,conArray,25,cmap=cm.jet)
        plt.contour(xPlumePlot,yPlumePlot,conArray_estimated,25,cmap=cm.jet)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")


        # Plot UAV
        if UAV_pose.pose.position.x != 0.0:
            xPltUAV.append(UAV_pose.pose.position.x)
            yPltUAV.append(UAV_pose.pose.position.y)
        if particleColor == "red":
            plt.plot(xPltUAV, yPltUAV,"r")
            plt.plot(UAV_pose.pose.position.x,UAV_pose.pose.position.y,'ro',  markersize=6)
        elif particleColor == "green":
            plt.plot(xPltUAV, yPltUAV,"g")
            plt.plot(UAV_pose.pose.position.x,UAV_pose.pose.position.y,'go',  markersize=6)
        elif particleColor == "blue":
            plt.plot(xPltUAV, yPltUAV,"b")
            plt.plot(UAV_pose.pose.position.x,UAV_pose.pose.position.y,'bo',  markersize=6)

        plt.xlim(xMinMap, xMaxMap)
        plt.ylim(yMinMap, yMaxMap)
        rate.sleep()
        plt.pause(0.05)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
