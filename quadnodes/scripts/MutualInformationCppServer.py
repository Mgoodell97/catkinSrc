#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from particle_filter.msg import particles
from quadnodes.srv import mutualInfo

# Packages
import numpy as np
import matplotlib.pyplot as plt
import pickle
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

state_cb_flag = False;
pose_cb_flag = False;

##################
# Functions
##################

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

##################
# Main
##################

def main():
    rospy.init_node('MutualInformationMotionPlanner')

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("true_position", PoseStamped, pose_cb)

    global_waypoint_pub = rospy.Publisher('desired_waypoint', PoseStamped, queue_size=100)

    maxVelocity        = rospy.get_param("MutualInformation/maxVelocity")     #  m/s
    stepSize           = rospy.get_param("MutualInformation/stepSize")        #  m
    waypointRadius     = rospy.get_param("MutualInformation/waypointRadius")  #  m
    stayTime           = rospy.get_param("MutualInformation/stayTime")        #  seconds
    xMinMap            = rospy.get_param("xMinMap")                           #  m
    xMaxMap            = rospy.get_param("xMaxMap")                           #  m
    yMinMap            = rospy.get_param("yMinMap")                           #  m
    yMaxMap            = rospy.get_param("yMaxMap")                           #  m
    zMinMap            = rospy.get_param("zMinMap")                           #  m
    zMaxMap            = rospy.get_param("zMaxMap")                           #  m

    rospy.wait_for_service('computeMutualInfoAtLoc') # wait for service
    info_client        = rospy.ServiceProxy("computeMutualInfoAtLoc", mutualInfo)

    xyzError = [0, 0, 0]
    justHitWaypoint = False
    optimalTimeToWaypoint = 100

    DesiredWaypoint = PoseStamped()

    waypointStartTime = rospy.get_rostime()
    optimalTimeToWaypointTimer = rospy.get_rostime()

    rate = rospy.Rate(2)

    while ((not rospy.is_shutdown() and current_state.connected) or not state_cb_flag or not pose_cb_flag):
        rate.sleep()
        if state_cb_flag and pose_cb_flag:
            break

    last_request = rospy.get_rostime()

    # Start first waypoint right above the robot
    xWaypoint = current_pose.pose.position.x
    yWaypoint = current_pose.pose.position.y
    zWaypoint = (zMaxMap+zMinMap)/2

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
                # print("")
                # print("Computing next waypoint")
                # print("")
                # print("=======================")

                zTestsHeightsSize = 3

                zRange = 0.1
                zTestsHeights = np.linspace(-zRange, zRange, num=zTestsHeightsSize) + current_pose.pose.position.z


                # zTestsHeights = np.array([2])

                desiredInfoVec = []
                infoVec = []

                for z in range(zTestsHeightsSize):
                    for i in range(thetaVecSize):
                        desiredInfoVec.append([current_pose.pose.position.x + stepSize*cos(thetaVec[i]), current_pose.pose.position.y + stepSize*sin(thetaVec[i]), zTestsHeights[z]])

                desiredInfoVec = np.array(desiredInfoVec)

                for i in range(len(desiredInfoVec)):
                    xInfoDes = desiredInfoVec[i,:]
                    # infoVec.append(informationAtXNew(xInfoDes,xp,wp,sigma,ztMinMain, ztMaxMain, xGuass, wGuass))

                    result = info_client(float(xInfoDes[0]),float(xInfoDes[1]),float(xInfoDes[2]))
                    infoVec.append(result.mutualInfoResponse)

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
            zTestsHeightsSize = 3

            zRange = 0.1
            zTestsHeights = np.linspace(-zRange, zRange, num=zTestsHeightsSize) + current_pose.pose.position.z

            # zTestsHeights = np.array([2])
            desiredInfoVec = []
            infoVec = []

            for z in range(zTestsHeightsSize):
                for i in range(thetaVecSize):
                    desiredInfoVec.append([current_pose.pose.position.x + stepSize*cos(thetaVec[i]), current_pose.pose.position.y + stepSize*sin(thetaVec[i]), zTestsHeights[z]])

            desiredInfoVec = np.array(desiredInfoVec)

            for i in range(len(desiredInfoVec)):
                xInfoDes = desiredInfoVec[i,:]
                # infoVec.append(informationAtXNew(xInfoDes,xp,wp,sigma,ztMinMain, ztMaxMain, xGuass, wGuass))

                result = info_client(float(xInfoDes[0]),float(xInfoDes[1]),float(xInfoDes[2]))
                infoVec.append(result.mutualInfoResponse)

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

        if zWaypoint >= zMaxMap:
            zWaypoint = zMaxMap
        elif zWaypoint <= zMinMap:
            zWaypoint = zMinMap

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
