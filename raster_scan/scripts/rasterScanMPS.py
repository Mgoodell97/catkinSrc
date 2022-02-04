#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from math import sqrt, pi
import numpy as np
import pickle

from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from mps_driver.msg import MPS

from tf.transformations import quaternion_from_euler
import tf_conversions
from GaussianSensorPackage import accountForSensorDynamics

##################
# Global variables
##################

Robot_pose = PoseStamped()
ppm_reading = 0
zPast = 0
yPast = 0

##################
# Functions
##################

def rasterScanGen(xRange, xNumSteps, yRange, yNumSteps):
    xArray = np.linspace(xRange[0], xRange[1], xNumSteps, endpoint=True)
    yArray = np.linspace(yRange[0], yRange[1], yNumSteps, endpoint=True)


    xScan = []
    yScan = []

    for i, yi in enumerate(yArray):
        xScan.append(xArray[::(-1)**i]) # reverse when i is odd
        yScan.append(np.ones_like(xArray) * yi)

    # squeeze lists together to vectors
    xScan = np.concatenate(xScan)
    yScan = np.concatenate(yScan)

    desiredWaypoints = np.array([xScan, yScan]).T

    return desiredWaypoints

def rasterScanGenFlipedXY(xRange, xNumSteps, yRange, yNumSteps):
    xArray = np.linspace(xRange[0], xRange[1], xNumSteps, endpoint=True)
    yArray = np.linspace(yRange[0], yRange[1], yNumSteps, endpoint=True)


    xScan = []
    yScan = []

    for i, yi in enumerate(yArray):
        xScan.append(xArray[::(-1)**i]) # reverse when i is odd
        yScan.append(np.ones_like(xArray) * yi)

    # squeeze lists together to vectors
    xScan = np.concatenate(xScan)
    yScan = np.concatenate(yScan)

    desiredWaypoints = np.array([yScan, xScan]).T

    return desiredWaypoints

##################
# Callbacks
##################

def Robot_pose_cb(pose_cb_msg):
    global Robot_pose
    Robot_pose = pose_cb_msg

def MPS_cb(MPS_cb_msg):
    global ppm_reading
    global zPast
    global yPast
    ppm_reading, zPast, yPast = accountForSensorDynamics(MPS_cb_msg.pressure, zPast, yPast, 1)

##################
# Main
##################

def main():
    rospy.init_node('rasterScan')

    global_waypoint_pub = rospy.Publisher('desiredPos', PoseStamped, queue_size=100)

    xmin      = rospy.get_param("robotPoseRaster/xmin")
    xmax      = rospy.get_param("robotPoseRaster/xmax")
    Nx        = rospy.get_param("robotPoseRaster/Nx")
    ymin      = rospy.get_param("robotPoseRaster/ymin")
    ymax      = rospy.get_param("robotPoseRaster/ymax")
    Ny        = rospy.get_param("robotPoseRaster/Ny")
    stayTime  = rospy.get_param("robotPoseRaster/stayTime")
    flipXY    = rospy.get_param("robotPoseRaster/flipXY")
    waypointRadius = rospy.get_param("robotPoseRaster/waypointRadius")  #  m
    startTest = rospy.get_param("/startTest")

    RobotID           = rospy.get_param("RobotID")        #  seconds

    fullStringName = "/mocap_node/Robot_" + str(int(RobotID)) + "/pose"

    rospy.Subscriber(fullStringName, PoseStamped, Robot_pose_cb)
    rospy.Subscriber("mps_data", MPS, MPS_cb)

    waypointIndex = 0
    if flipXY:
        desiredWaypointsList = rasterScanGenFlipedXY([xmin, xmax], Nx, [ymin, ymax], Ny)
    else:
        desiredWaypointsList = rasterScanGen([xmin, xmax], Nx, [ymin, ymax], Ny)

    xyzError = [0, 0, 0]
    justHitWaypoint = False
    firstWaypointFlag = False
    optimalTimeToWaypoint = 20
    maxVelocity = 0.075

    DesiredWaypoint = PoseStamped()

    waypointStartTime = rospy.get_rostime()
    optimalTimeToWaypointTimer = rospy.get_rostime()

    rate = rospy.Rate(30)

    # Start first waypoint right above the robot
    xWaypoint = desiredWaypointsList[waypointIndex,0]
    yWaypoint = desiredWaypointsList[waypointIndex,1]
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0.785398)

    while (not rospy.is_shutdown() or not rospy.get_param("/startTest")):
        DesiredWaypoint.pose.position.x = xWaypoint
        DesiredWaypoint.pose.position.y = yWaypoint
        DesiredWaypoint.pose.position.z = 0 # doesn't matter for omni wheel robot (whats it going to do fly?? Am I right? sighhhhh)
        DesiredWaypoint.pose.orientation.x = q[0]
        DesiredWaypoint.pose.orientation.y = q[1]
        DesiredWaypoint.pose.orientation.z = q[2]
        DesiredWaypoint.pose.orientation.w = q[3]
        global_waypoint_pub.publish(DesiredWaypoint);
        rate.sleep()
        if rospy.get_param("/startTest"):
            break

    concentrationReadingList = [] # fill in list [x, y, con]
    currentConList = []
    while not rospy.is_shutdown():
        # print(concentrationReadingList)

        xyzError[0] = xWaypoint - Robot_pose.pose.position.x
        xyzError[1] = yWaypoint - Robot_pose.pose.position.y
        xyzError[2] = 0

        withinWaypoint = sqrt(pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2))

        if(withinWaypoint <= waypointRadius or (rospy.get_rostime() - optimalTimeToWaypointTimer >= rospy.Duration(optimalTimeToWaypoint*2))):
            currentConList.append(ppm_reading)
            if( not justHitWaypoint): # Start timing waypoint
                waypointStartTime = rospy.get_rostime()
                justHitWaypoint = True;

            if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)): # move to next waypoint
                meanConReading = np.mean(currentConList)
                concentrationReadingList.append([xWaypoint, yWaypoint, meanConReading])
                print([xWaypoint, yWaypoint, meanConReading])
                currentConList = []

                waypointIndex +=1
                optimalTimeToWaypointTimer = rospy.get_rostime()

                if waypointIndex == len(desiredWaypointsList):
                    # print("Waypoint list completed")
                    break

                # if waypointIndex == len(desiredWaypointsList):
                #     waypointIndex = 0

                xWaypoint = desiredWaypointsList[waypointIndex,0]
                yWaypoint = desiredWaypointsList[waypointIndex,1]

                waypointDistance = sqrt( (xWaypoint - Robot_pose.pose.position.x)**2 + (yWaypoint - Robot_pose.pose.position.y)**2 )

                optimalTimeToWaypoint = waypointDistance/maxVelocity

                # print("")
                # print("Moving to next waypoint")
                # print("")
                # print("=======================")

                justHitWaypoint = False

        else: # not at waypoint
            currentConList = [] # Reset concentration reading list
            justHitWaypoint = False
            # optimalTimeToWaypointTimer = rospy.get_rostime()

        DesiredWaypoint.pose.position.x = xWaypoint
        DesiredWaypoint.pose.position.y = yWaypoint
        DesiredWaypoint.pose.position.z = 0 # doesn't matter for omni wheel robot (whats it going to do fly?? Am I right? sighhhhh)
        DesiredWaypoint.pose.orientation.x = q[0]
        DesiredWaypoint.pose.orientation.y = q[1]
        DesiredWaypoint.pose.orientation.z = q[2]
        DesiredWaypoint.pose.orientation.w = q[3]



        global_waypoint_pub.publish(DesiredWaypoint);

        rate.sleep()

    # print("Saving MPS waypoint data")
    db = {}
    db["MPSdataList"]   = concentrationReadingList
    pickleStringName = "MPS_data_Robot_" + str(int(RobotID))
    # dbfile = open(pickleStringName, 'ab')
    # pickle.dump(db, dbfile)
    # dbfile.close()
    pickle.dump( db, open(pickleStringName, "wb" ) )
    print("MPS data saved for robot ", str(int(RobotID)))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
