#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from geometry_msgs.msg import TwistStamped
from particle_filter.msg import particles
import matplotlib.pyplot as plt
import pickle

# Packages
import numpy as np
from math import cos, sin, pi, acos, sqrt, exp
from scipy.special import roots_legendre
from scipy.stats import norm

##################
# Global variables
##################

global current_state
global current_pose
current_pose = PoseStamped()
current_state = State()
allParticles = particles()

from particle_filter.msg import particles

state_cb_flag = False;
pose_cb_flag = False;
pf_cb_flag = False;

xGuass, wGuass = roots_legendre(5)

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

def conditionalEntropyAndMeasurementEntropy(xInfoDes,zt,particlesFunc,sigma):
    wp = np.array(particlesFunc.weights)
    particleObsravtionAtXDes = np.zeros(len(wp))
    for j in range(len(wp)):
        particleObsravtionAtXDes[j] = getReading(xInfoDes[0],xInfoDes[1], particlesFunc.theta[j], particlesFunc.X[j], particlesFunc.Y[j], particlesFunc.X[j] - current_pose.pose.position.z, particlesFunc.Q[j], particlesFunc.v[j], particlesFunc.Dy[j], particlesFunc.Dz[j])

    partileLikelihood = norm.pdf(zt, particleObsravtionAtXDes, sigma)

    probabilityZt = wp * partileLikelihood
    probabilityZtSum = np.sum(probabilityZt)

    measurementEntropySum = probabilityZtSum * np.log(probabilityZtSum)

    conditionalEntropySum = np.sum(probabilityZtSum * np.log(partileLikelihood))

    return -measurementEntropySum, -conditionalEntropySum

def informationAtXNew(xInfoDes, particlesFunc, sigma, ztMin, ztMax, xGuass, wGuass):
    integralConditional = 0
    integralMeasurment = 0
    for i in range(len(xGuass)):
        convertedZt = ((ztMin-ztMax)/2) * xGuass[i] + ((ztMin+ztMax)/2)
        measurementEntropySum, conditionalEntropySum = conditionalEntropyAndMeasurementEntropy(xInfoDes,convertedZt,particlesFunc,sigma)
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
    global allParticles
    global pf_cb_flag
    allParticles = pfMsg
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

    zHeight            = rospy.get_param("MutualInformation/zHeight")         #  m
    stepSize           = rospy.get_param("MutualInformation/stepSize")        #  m
    waypointRadius     = rospy.get_param("MutualInformation/waypointRadius")  #  m
    stayTime           = rospy.get_param("MutualInformation/stayTime")        #  seconds
    xMinMap            = rospy.get_param("xMinMap")                           #  m
    xMaxMap            = rospy.get_param("xMaxMap")                           #  m
    yMinMap            = rospy.get_param("yMinMap")                           #  m
    yMaxMap            = rospy.get_param("yMaxMap")                           #  m

    xyzError = [0, 0, 0]
    justHitWaypoint = False
    sigma = 10000
    ztMinMain = 0
    ztMaxMain = 125000
    pickleFlag = True

    DesiredWaypoint = PoseStamped()

    waypointStartTime = rospy.get_rostime()

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

    thetaVecSize = 12
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

                if pickleFlag: # for offline testing
                    particleArray = [allParticles.X,allParticles.Y,allParticles.Z,allParticles.theta,allParticles.Q,allParticles.v,allParticles.Dy,allParticles.Dz,allParticles.weights]
                    filename = 'pickled_array.p'
                    with open(filename, 'wb') as filehandler:
                        pickle.dump(particleArray, filehandler)

                    pickleFlag = False

                zTestsHeightsSize = 1

                # zRange = 0.1
                # zTestsHeights = np.linspace(-zRange, zRange, num=zTestsHeightsSize) + current_pose.pose.position.z

                zTestsHeights = np.array([2])
                desiredInfoVec = []
                infoVec = []

                tempPos = current_pose

                for z in range(zTestsHeightsSize):
                    for i in range(thetaVecSize):
                        desiredInfoVec.append([tempPos.pose.position.x + stepSize*cos(thetaVec[i]), tempPos.pose.position.y + stepSize*sin(thetaVec[i]), zTestsHeights[z]])

                desiredInfoVec = np.array(desiredInfoVec)

                for i in range(len(desiredInfoVec)):
                    xInfoDes = desiredInfoVec[i,:]
                    infoVec.append(informationAtXNew(xInfoDes,allParticles,sigma,ztMinMain, ztMaxMain, xGuass, wGuass))




                infoVec = np.array(infoVec)

                waypoint = desiredInfoVec[np.argmax(infoVec),:]

                # print(desiredInfoVec)

                xWaypoint = waypoint[0]
                yWaypoint = waypoint[1]
                zWaypoint = waypoint[2]

                print("")
                print("Moving to next waypoint")
                print(waypoint)
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

        try:
            plt.clf()

            plt.scatter(desiredInfoVec[:,0], desiredInfoVec[:,1], marker='o', c=infoVec, cmap="jet")
            plt.axis('square')
            plt.xlabel("x [m]")
            plt.ylabel("y [m]")

            plt.pause(0.01)
        except:
            pass

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
