#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from math import sqrt, pi
import numpy as np

from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

from particle_filter.msg import estimatedGaussian

from BRW_functions import biasedRandomWalk, moveRobot
from GaussianSensorPackage import GaussianSensor

##################
# Global variables
##################

global gaussEstimateFlag
global current_gaussEstimate
global Robot_pose

estimatedPlume = estimatedGaussian()
Robot_pose = PoseStamped()

gaussEstimateFlag = False;

##################
# Functions
##################

##################
# Callbacks
##################

def gauss_PF_Estimate(gaussEstimate):
    global estimatedPlume
    global gaussEstimateFlag
    estimatedPlume = gaussEstimate
    gaussEstimateFlag = True

def Robot_pose_cb(pose_cb_msg):
    global Robot_pose
    Robot_pose = pose_cb_msg

##################
# Main
##################

def main():
    rospy.init_node('BRW')

    global_waypoint_pub = rospy.Publisher('desiredPos', PoseStamped, queue_size=100)

    minLimX             = rospy.get_param("BRW/minLimX")          #  m
    maxLimX             = rospy.get_param("BRW/maxLimX")          #  m
    minLimY             = rospy.get_param("BRW/minLimY")          #  m
    maxLimY             = rospy.get_param("BRW/maxLimY")          #  m
    biasRange          = rospy.get_param("BRW/biasRange")       #  degrees
    stepSize           = rospy.get_param("BRW/stepSize")        #  m
    waypointRadius     = rospy.get_param("BRW/waypointRadius")  #  m
    stayTime           = rospy.get_param("BRW/stayTime")        #  seconds

    RobotID           = rospy.get_param("RobotID")        #  seconds

    fullStringName = "/mocap_node/Robot_" + str(int(RobotID)) + "/pose"

    rospy.Subscriber(fullStringName, PoseStamped, Robot_pose_cb)
    rospy.Subscriber("estimatedGaussian", estimatedGaussian, gauss_PF_Estimate)

    biasRange = biasRange * pi/180 # converts to radians
    xyzError = [0, 0, 0]
    justHitWaypoint = False
    firstWaypointFlag = False
    optimalTimeToWaypoint = 20
    maxVelocity = 0.1

    DesiredWaypoint = PoseStamped()

    waypointStartTime = rospy.get_rostime()
    optimalTimeToWaypointTimer = rospy.get_rostime()

    rate = rospy.Rate(30)

    while (not gaussEstimateFlag or not rospy.is_shutdown() or not rospy.get_param("/startTest")):
        rate.sleep()
        if gaussEstimateFlag and rospy.get_param("/startTest"):
            break

    # Start first waypoint right above the robot
    xWaypoint = Robot_pose.pose.position.x
    yWaypoint = Robot_pose.pose.position.y

    while not rospy.is_shutdown():
        xyzError[0] = xWaypoint - Robot_pose.pose.position.x
        xyzError[1] = yWaypoint - Robot_pose.pose.position.y
        xyzError[2] = 0

        withinWaypoint = sqrt(pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2))

        if(withinWaypoint <= waypointRadius or (rospy.get_rostime() - optimalTimeToWaypointTimer >= rospy.Duration(optimalTimeToWaypoint*1.5))):
            if( not justHitWaypoint):
                waypointStartTime = rospy.get_rostime()
                justHitWaypoint = True;
            if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)):
                if not firstWaypointFlag:
                    # Get first reading
                    previousReading = GaussianSensor(Robot_pose.pose.position.x, Robot_pose.pose.position.y, estimatedPlume.Theta, estimatedPlume.X, estimatedPlume.Y, 0, estimatedPlume.Q, estimatedPlume.V, estimatedPlume.Dy, estimatedPlume.Dz)
                    xRobotDesired, yRobotDesired = moveRobot(Robot_pose.pose.position.x, Robot_pose.pose.position.y, stepSize, minLimX, maxLimX, minLimY, maxLimY)

                    xWaypoint = xRobotDesired
                    yWaypoint = yRobotDesired

                    previousBias = np.arctan2((yRobotDesired-Robot_pose.pose.position.y),(xRobotDesired-Robot_pose.pose.position.x))

                    #Only move randomly once
                    firstWaypointFlag = True
                    optimalTimeToWaypointTimer = rospy.get_rostime()

                else: # Start bias random walk
                    currentReading = GaussianSensor(Robot_pose.pose.position.x, Robot_pose.pose.position.y, estimatedPlume.Theta, estimatedPlume.X, estimatedPlume.Y, 0, estimatedPlume.Q, estimatedPlume.V, estimatedPlume.Dy, estimatedPlume.Dz)
                    xRobotDesired, yRobotDesired, slope, bias = biasedRandomWalk(Robot_pose.pose.position.x, Robot_pose.pose.position.y, previousReading, currentReading, biasRange, previousBias, stepSize, minLimX, maxLimX, minLimY, maxLimY)

                    xWaypoint = xRobotDesired
                    yWaypoint = yRobotDesired

                    previousReading = currentReading
                    previousBias = bias
                    optimalTimeToWaypointTimer = rospy.get_rostime()


                waypointDistance = sqrt( (xWaypoint - Robot_pose.pose.position.x)**2 + (yWaypoint - Robot_pose.pose.position.y)**2 )

                optimalTimeToWaypoint = waypointDistance/maxVelocity

                # print("")
                # print("Moving to next waypoint")
                # print("")
                # print("=======================")

                justHitWaypoint = False

        else:
            justHitWaypoint = False
            # optimalTimeToWaypointTimer = rospy.get_rostime()

        DesiredWaypoint.pose.position.x = xWaypoint
        DesiredWaypoint.pose.position.y = yWaypoint
        DesiredWaypoint.pose.position.z = 0 # doesn't matter for omni wheel robot (whats it going to do fly?? Am I right? sighhhhh)



        global_waypoint_pub.publish(DesiredWaypoint);

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
