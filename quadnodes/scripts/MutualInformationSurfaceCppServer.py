#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from particle_filter.msg import particles
from quadnodes.srv import mutualInfo

# Packages
import numpy as np
import matplotlib.pyplot as plt
import pickle
from math import cos, sin, pi, acos, sqrt, exp
from scipy.special import roots_legendre
from scipy.stats import norm
from numpy import savetxt

##################
# Global variables
##################

##################
# Functions
##################

##################
# Callbacks
##################

##################
# Main
##################

def main():
    rospy.init_node('MutualInformationSurfacePlotNew')


    rospy.wait_for_service('computeMutualInfoAtLoc') # wait for service
    info_client        = rospy.ServiceProxy("computeMutualInfoAtLoc", mutualInfo)


    xMinMap            = rospy.get_param("xMinMap")                           #  m
    xMaxMap            = rospy.get_param("xMaxMap")                           #  m
    yMinMap            = rospy.get_param("yMinMap")                           #  m
    yMaxMap            = rospy.get_param("yMaxMap")                           #  m

    rate = rospy.Rate(1)

    fig, ax = plt.subplots()
    while not rospy.is_shutdown():
        plt.clf()

        xLinspaceVec = np.linspace(xMinMap, xMaxMap, num=25)
        yLinspaceVec = np.linspace(yMinMap, yMaxMap, num=25)

        infoArray = np.zeros([len(yLinspaceVec), len(xLinspaceVec)])

        for yCurrentIndex in range(len(yLinspaceVec)):
            for xCurrentIndex in range(len(xLinspaceVec)):
                # rospy.wait_for_service('computeMutualInfoAtLoc') # wait for service
                xInfoDes = np.array([xLinspaceVec[xCurrentIndex], yLinspaceVec[yCurrentIndex], 0])
                                                                        # x          y         z
                result = info_client(float(xInfoDes[0]),float(xInfoDes[1]),float(xInfoDes[2]))
                infoArray[yCurrentIndex,xCurrentIndex] = result.mutualInfoResponse
                # infoArray[yCurrentIndex,xCurrentIndex] = informationAtXNew(xInfoDes,xp,wp,sigma,ztMinMain, ztMaxMain, xGuass, wGuass)

        # print(wp)

        plt.subplot("111")
        plt.contourf(xLinspaceVec, yLinspaceVec, infoArray, levels=100)
        # plt.plot(xp[:,0],xp[:,1],'r.')
        plt.xlim(xMinMap, xMaxMap)
        plt.ylim(yMinMap, yMaxMap)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.xlabel("Cpp server")

        plt.pause(0.01)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
