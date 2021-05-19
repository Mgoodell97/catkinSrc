#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from geometry_msgs.msg import TwistStamped
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

##################
# Global variables
##################

global current_state
global current_pose
global condensedParticleArray
global n_clusters

current_pose = PoseStamped()
current_state = State()
# allParticles = particles()

state_cb_flag = False;
pose_cb_flag = False;
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

    condensedParticles[:,-1] = condensedParticles[:,-1]/sum(condensedParticles[:,-1]) # normalize weights

    return condensedParticles

def gaussFunc(xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc + (zFunc**2)/DzFunc))
    # if np.isnan(con):
    #     con = 0
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

def pose_cb(poseMsg):
    global current_pose
    global pose_cb_flag
    current_pose = poseMsg
    pose_cb_flag = True

def state_cb(stateMsg):
    global current_state
    global state_cb_flag
    current_state = stateMsg
    state_cb_flag = True

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
    rospy.init_node('MutualInformation')

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("true_position", PoseStamped, pose_cb)
    rospy.Subscriber("particles", particles, pf_cb) # Sub to particle filter

    global_waypoint_pub = rospy.Publisher('desired_waypoint', PoseStamped, queue_size=100)

    maxVelocity        = rospy.get_param("MutualInformation/maxVelocity")     #  m/s
    zHeight            = rospy.get_param("MutualInformation/zHeight")         #  m
    stepSize           = rospy.get_param("MutualInformation/stepSize")        #  m
    waypointRadius     = rospy.get_param("MutualInformation/waypointRadius")  #  m
    stayTime           = rospy.get_param("MutualInformation/stayTime")        #  seconds
    xMinMap            = rospy.get_param("xMinMap")                           #  m
    xMaxMap            = rospy.get_param("xMaxMap")                           #  m
    yMinMap            = rospy.get_param("yMinMap")                           #  m
    yMaxMap            = rospy.get_param("yMaxMap")                           #  m
    sigma              = rospy.get_param("sigma")                             #  unitless
    ztMinMain          = rospy.get_param("ztMinMain")                         #  ppm
    ztMaxMain          = rospy.get_param("ztMaxMain")                         #  ppm

    xyzError = [0, 0, 0]
    justHitWaypoint = False
    pickleFlag = True
    optimalTimeToWaypoint = 100

    DesiredWaypoint = PoseStamped()

    waypointStartTime = rospy.get_rostime()
    optimalTimeToWaypointTimer = rospy.get_rostime()

    rate = rospy.Rate(1)

    while ((not rospy.is_shutdown() and current_state.connected) or not state_cb_flag or not pose_cb_flag or not pf_cb_flag):
        rate.sleep()
        if state_cb_flag and pose_cb_flag and pf_cb_flag:
            break

    last_request = rospy.get_rostime()

    # Start first waypoint right above the robot
    xWaypoint = current_pose.pose.position.x
    yWaypoint = current_pose.pose.position.y
    zWaypoint = zHeight

    thetaVecSize = 16
    thetaVec = np.linspace(0, 2*pi - (2*pi/thetaVecSize), num=thetaVecSize)

    while not rospy.is_shutdown():
        xyzError[0] = xWaypoint - current_pose.pose.position.x
        xyzError[1] = yWaypoint - current_pose.pose.position.y
        xyzError[2] = zWaypoint - current_pose.pose.position.z

        withinWaypoint = sqrt(pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2))

        if(withinWaypoint <= waypointRadius):
            if( not justHitWaypoint):
                waypointStartTime = rospy.get_rostime()
                justHitWaypoint = True;
            if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)): # Go to new waypoint
                print("")
                print("Computing next waypoint")
                print("")
                print("=======================")

                # if pickleFlag: # for offline testing
                #     particleArray = [allParticles.X,allParticles.Y,allParticles.Z,allParticles.theta,allParticles.Q,allParticles.v,allParticles.Dy,allParticles.Dz,allParticles.weights]
                #     filename = 'pickled_array.p'
                #     with open(filename, 'wb') as filehandler:
                #         pickle.dump(particleArray, filehandler)
                #
                #     pickleFlag = False

                zTestsHeightsSize = 1

                # zRange = 0.1
                # zTestsHeights = np.linspace(-zRange, zRange, num=zTestsHeightsSize) + current_pose.pose.position.z

                zTestsHeights = np.array([2])
                desiredInfoVec = []
                infoVec = []

                for z in range(zTestsHeightsSize):
                    for i in range(thetaVecSize):
                        desiredInfoVec.append([current_pose.pose.position.x + stepSize*cos(thetaVec[i]), current_pose.pose.position.y + stepSize*sin(thetaVec[i]), zTestsHeights[z]])

                desiredInfoVec = np.array(desiredInfoVec)

                global condensedParticleArray

                xp = condensedParticleArray[:,0:8]
                wp = condensedParticleArray[:,8]


                for i in range(len(desiredInfoVec)):
                    xInfoDes = desiredInfoVec[i,:]
                    infoVec.append(informationAtXNew(xInfoDes,xp,wp,sigma,ztMinMain, ztMaxMain, xGuass, wGuass))

                infoVec = np.array(infoVec)

                waypoint = desiredInfoVec[np.argmax(infoVec),:]

                # print(desiredInfoVec)

                xWaypoint = waypoint[0]
                yWaypoint = waypoint[1]
                zWaypoint = waypoint[2]

                waypointDistance = sqrt( (xWaypoint - current_pose.pose.position.x)**2 + (yWaypoint - current_pose.pose.position.y)**2 )
                optimalTimeToWaypoint = waypointDistance/maxVelocity
                optimalTimeToWaypointTimer = rospy.get_rostime() # reset timer

                print("")
                print("Moving to next waypoint")
                print(waypoint)
                print("")
                print("=======================")

                justHitWaypoint = False
        elif(rospy.get_rostime() - optimalTimeToWaypointTimer >= rospy.Duration(optimalTimeToWaypoint*1.5)):
            zTestsHeightsSize = 1

            # zRange = 0.1
            # zTestsHeights = np.linspace(-zRange, zRange, num=zTestsHeightsSize) + current_pose.pose.position.z

            zTestsHeights = np.array([2])
            desiredInfoVec = []
            infoVec = []

            for z in range(zTestsHeightsSize):
                for i in range(thetaVecSize):
                    desiredInfoVec.append([current_pose.pose.position.x + stepSize*cos(thetaVec[i]), current_pose.pose.position.y + stepSize*sin(thetaVec[i]), zTestsHeights[z]])

            desiredInfoVec = np.array(desiredInfoVec)

            global condensedParticleArray

            xp = condensedParticleArray[:,0:8]
            wp = condensedParticleArray[:,8]


            for i in range(len(desiredInfoVec)):
                xInfoDes = desiredInfoVec[i,:]
                infoVec.append(informationAtXNew(xInfoDes,xp,wp,sigma,ztMinMain, ztMaxMain, xGuass, wGuass))

            infoVec = np.array(infoVec)

            waypoint = desiredInfoVec[np.argmax(infoVec),:]

            # print(desiredInfoVec)

            xWaypoint = waypoint[0]
            yWaypoint = waypoint[1]
            zWaypoint = waypoint[2]

            waypointDistance = sqrt( (xWaypoint - current_pose.pose.position.x)**2 + (yWaypoint - current_pose.pose.position.y)**2 )
            optimalTimeToWaypoint = waypointDistance/maxVelocity
            optimalTimeToWaypointTimer = rospy.get_rostime() # reset timer

            print("")
            print("Could not reach waypoint moving to next waypoint")
            print("waypointDistance      : ", waypointDistance)
            print("optimalTimeToWaypoint : ", optimalTimeToWaypoint)
            print("")
            print("=======================")

            justHitWaypoint = False



        else:
            justHitWaypoint = False


        if xWaypoint >= xMaxMap:
            xWaypoint = xMaxMap
        elif xWaypoint <= xMinMap:
            xWaypoint = xMinMap

        if yWaypoint >= yMaxMap:
            yWaypoint = yMaxMap
        elif yWaypoint <= yMinMap:
            yWaypoint = yMinMap


        DesiredWaypoint.pose.position.x = xWaypoint
        DesiredWaypoint.pose.position.y = yWaypoint
        DesiredWaypoint.pose.position.z = zWaypoint
        global_waypoint_pub.publish(DesiredWaypoint);

        # try:
        #     plt.clf()
        #
        #     plt.scatter(desiredInfoVec[:,0], desiredInfoVec[:,1], marker='.', c=infoVec, cmap="jet")
        #     plt.plot([current_pose.pose.position.x, xWaypoint],[current_pose.pose.position.y, yWaypoint])
        #     plt.axis('square')
        #     plt.xlabel("x [m]")
        #     plt.ylabel("y [m]")
        #
        #     plt.pause(0.01)
        # except:
        #     pass

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
