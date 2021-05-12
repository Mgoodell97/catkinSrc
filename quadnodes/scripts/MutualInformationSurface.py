#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from particle_filter.msg import particles

# Packages
import numpy as np
import matplotlib.pyplot as plt
import pickle
from math import cos, sin, pi, acos, sqrt, exp
from sklearn.metrics.pairwise import pairwise_distances_argmin
from sklearn.cluster import MiniBatchKMeans
from scipy.special import roots_legendre
from scipy.stats import norm
from numpy import savetxt

##################
# Global variables
##################

global condensedParticleArray
global n_clusters

pf_cb_flag = False;

xGuass, wGuass = roots_legendre(5)
n_clusters = 50
condensedParticleArray = np.zeros((n_clusters, 9)) # X, Y, Z, theta, Q, v, Dy, Dz, weights


##################
# Functions
##################

def condenseParticles(particleArray, n_clusters):
    X = particleArray[:,0:2]
    k_means = MiniBatchKMeans(init='k-means++', n_clusters=n_clusters, n_init=10, max_no_improvement=10)
    k_means.fit(X) # Fit particles

    condensedParticles = np.zeros((n_clusters, particleArray.shape[1]))
    k_means_cluster_centers = k_means.cluster_centers_
    k_means_labels = pairwise_distances_argmin(X, k_means_cluster_centers)

    for k in range(n_clusters):
        my_members = k_means_labels == k
        groupedParticles = particleArray[my_members]
        condensedParticles[k,:] = np.mean(groupedParticles, axis=0)

    condensedParticles[:,-1] = condensedParticles[:,-1]/sum(condensedParticles[:,-1])

    return condensedParticles

def gaussFunc(xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc + (zFunc**2)/DzFunc))
    return con

def getReading(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    # Rotate frame

    Xw_r = np.array([xRobotDef, yRobotDef, 1])

    R = np.array([[cos(thetaFunc), -sin(thetaFunc)], [sin(thetaFunc), cos(thetaFunc)]])
    P = np.array([[xPlumeFunc, yPlumeFunc]]).T;

    bottomArray = np.array([[0, 0 , 1]]);
    Tw_plumeFrame = np.concatenate((R, P), axis=1)

    # Transfer matrix from plumeframe to w
    Tw_plumeFrame = np.concatenate((Tw_plumeFrame, bottomArray), axis=0)

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

def conditionalEntropyAndMeasurementEntropy(xInfoDes, zt, xp, wp, sigma):
    particleObsravtionAtXDes = np.zeros(len(wp))
    for j in range(len(wp)):
        particleObsravtionAtXDes[j] = getReading(xInfoDes[0],xInfoDes[1], xp[j][3], xp[j][0], xp[j][1], xp[j][2] - xInfoDes[2], xp[j][4], xp[j][5], xp[j][6], xp[j][7])

    partileLikelihood = norm.pdf(zt, particleObsravtionAtXDes, sigma)

    probabilityZt = wp * partileLikelihood
    probabilityZtSum = np.sum(probabilityZt)

    measurementEntropySum = probabilityZtSum * np.log(probabilityZtSum)

    conditionalEntropySum = np.sum(probabilityZtSum * np.log(partileLikelihood))

    return -measurementEntropySum, -conditionalEntropySum

def informationAtXNew(xInfoDes, xp, wp, sigma, ztMin, ztMax, xGuass, wGuass):
    integralConditional = 0
    integralMeasurment = 0
    for i in range(len(xGuass)):
        convertedZt = ((ztMin-ztMax)/2) * xGuass[i] + ((ztMin+ztMax)/2)
        measurementEntropySum, conditionalEntropySum = conditionalEntropyAndMeasurementEntropy(xInfoDes, convertedZt, xp, wp, sigma)
        integralMeasurment = integralMeasurment + wGuass[i] * measurementEntropySum
        integralConditional = integralConditional + wGuass[i] * conditionalEntropySum

    MutualInfo = integralConditional - integralMeasurment

    return MutualInfo

##################
# Callbacks
##################

def pf_cb(pfMsg):
    global pf_cb_flag
    global n_clusters
    global condensedParticleArray

    allParticles = pfMsg
    particleArray = np.array([allParticles.X,allParticles.Y,allParticles.Z,allParticles.theta,allParticles.Q,allParticles.v,allParticles.Dy,allParticles.Dz,allParticles.weights]).T
    condensedParticleArray = condenseParticles(particleArray, n_clusters)
    pf_cb_flag = True

##################
# Main
##################

def main():
    rospy.init_node('MutualInformationSurfacePlot')

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("particles", particles, pf_cb) # Sub to particle filter

    xMinMap            = rospy.get_param("xMinMap")                           #  m
    xMaxMap            = rospy.get_param("xMaxMap")                           #  m
    yMinMap            = rospy.get_param("yMinMap")                           #  m
    yMaxMap            = rospy.get_param("yMaxMap")                           #  m
    sigma              = rospy.get_param("sigma")                             #  unitless
    ztMinMain          = rospy.get_param("ztMinMain")                         #  ppm
    ztMaxMain          = rospy.get_param("ztMaxMain")                         #  ppm

    pickleFlag = True

    rate = rospy.Rate(1)

    while (not pf_cb_flag):
        rate.sleep()
        if pf_cb_flag:
            break


    fig, ax = plt.subplots()
    while not rospy.is_shutdown():
        plt.clf()



        global condensedParticleArray

        if pickleFlag: # for offline testing
            np.savetxt('condenseParticles.csv', condensedParticleArray, delimiter=',')
            # filename = 'condensedParticleArray.p'
            # with open(filename, 'wb') as filehandler:
            #     pickle.dump(condensedParticleArray, filehandler)

            pickleFlag = False

        # print(condensedParticleArray)
        xp = condensedParticleArray[:,0:8]
        wp = condensedParticleArray[:,8]

        xLinspaceVec = np.linspace(xMinMap, xMaxMap, num=15)
        yLinspaceVec = np.linspace(yMinMap, yMaxMap, num=15)

        infoArray = np.zeros([len(yLinspaceVec), len(xLinspaceVec)])

        for yCurrentIndex in range(len(yLinspaceVec)):
            for xCurrentIndex in range(len(xLinspaceVec)):
                xInfoDes = np.array([xLinspaceVec[xCurrentIndex], yLinspaceVec[yCurrentIndex], 2])
                infoArray[yCurrentIndex,xCurrentIndex] = informationAtXNew(xInfoDes,xp,wp,sigma,ztMinMain, ztMaxMain, xGuass, wGuass)

        # print(wp)

        plt.subplot("111")
        plt.contourf(xLinspaceVec, yLinspaceVec, infoArray, levels=100)
        plt.plot(xp[:,0],xp[:,1],'r.')
        plt.xlim(xMinMap, xMaxMap)
        plt.ylim(yMinMap, yMaxMap)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")

        plt.pause(0.01)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
