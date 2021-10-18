#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from math import sqrt, pi
import numpy as np

from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from quadnodes.msg import gaussian
from olfaction_msgs.msg import gas_sensor

from BRW_functions import biasedRandomWalk, moveRobot

##################
# Global variables
##################

global readingsFlag
global current_reading_full_data_gauss
global current_reading_full_data_gaden
current_reading_full_data_gauss = gaussian()
current_reading_full_data_gaden = gas_sensor()

readingsFlag = False;

##################
# Functions
##################

##################
# Callbacks
##################

def gaussSensor_cb(gaussMsg):
    global current_reading_full_data_gauss
    global readingsFlag
    current_reading_full_data_gauss = gaussMsg
    readingsFlag = True

def gadenSensor_cb(gadenMsg):
    global current_reading_full_data_gaden
    global readingsFlag
    current_reading_full_data_gaden = gadenMsg
    readingsFlag = True

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
    plumeType          = rospy.get_param("/plumeType")       #  I'm not explaining this

    if plumeType == 1:
        rospy.Subscriber("gaussianReading", gaussian, gaussSensor_cb)
    if plumeType == 2:
        rospy.Subscriber("Sensor_reading", gas_sensor, gadenSensor_cb)

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

    while (not readingsFlag and not rospy.is_shutdown()):
        rate.sleep()
        if readingsFlag:
            break

    # Start first waypoint right above the robot
    if plumeType == 1:
        xWaypoint = current_reading_full_data_gauss.x
        yWaypoint = current_reading_full_data_gauss.y
    elif plumeType == 2:
        xWaypoint = current_reading_full_data_gaden.local_x
        yWaypoint = current_reading_full_data_gaden.local_y



    while not rospy.is_shutdown():
        if plumeType == 1:
            xyzError[0] = xWaypoint - current_reading_full_data_gauss.x
            xyzError[1] = yWaypoint - current_reading_full_data_gauss.y
            xyzError[2] = 0
        if plumeType == 2:
            xyzError[0] = xWaypoint - current_reading_full_data_gaden.local_x
            xyzError[1] = yWaypoint - current_reading_full_data_gaden.local_y
            xyzError[2] = 0

        withinWaypoint = sqrt(pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2))



        if(withinWaypoint <= waypointRadius or (rospy.get_rostime() - optimalTimeToWaypointTimer >= rospy.Duration(optimalTimeToWaypoint*1.5))):
            if( not justHitWaypoint):
                waypointStartTime = rospy.get_rostime()
                justHitWaypoint = True;
            if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)):
                if not firstWaypointFlag:
                    # Get first reading
                    if plumeType == 1:
                        previousReading = current_reading_full_data_gauss.ppm
                        xRobotDesired, yRobotDesired = moveRobot(current_reading_full_data_gauss.x, current_reading_full_data_gauss.y, stepSize, minLimX, maxLimX, minLimY, maxLimY)
                    if plumeType == 2:
                        previousReading = current_reading_full_data_gaden.raw
                        xRobotDesired, yRobotDesired = moveRobot(current_reading_full_data_gaden.local_x, current_reading_full_data_gaden.local_y, stepSize, minLimX, maxLimX, minLimY, maxLimY)

                    xWaypoint = xRobotDesired
                    yWaypoint = yRobotDesired

                    if plumeType == 1:
                        previousBias = np.arctan2((yRobotDesired-current_reading_full_data_gauss.y),(xRobotDesired-current_reading_full_data_gauss.x))
                    if plumeType == 2:
                        previousBias = np.arctan2((yRobotDesired-current_reading_full_data_gaden.local_y),(xRobotDesired-current_reading_full_data_gaden.local_x))

                    #Only move randomly once
                    firstWaypointFlag = True
                    optimalTimeToWaypointTimer = rospy.get_rostime()

                else: # Start bias random walk
                    if plumeType == 1:
                        currentReading = current_reading_full_data_gauss.ppm
                        xRobotDesired, yRobotDesired, slope, bias = biasedRandomWalk(current_reading_full_data_gauss.x, current_reading_full_data_gauss.y, previousReading, currentReading, biasRange, previousBias, stepSize, minLimX, maxLimX, minLimY, maxLimY)
                    if plumeType == 2:
                        currentReading = current_reading_full_data_gaden.raw
                        xRobotDesired, yRobotDesired, slope, bias = biasedRandomWalk(current_reading_full_data_gaden.local_x, current_reading_full_data_gaden.local_y, previousReading, currentReading, biasRange, previousBias, stepSize, minLimX, maxLimX, minLimY, maxLimY)
                    xWaypoint = xRobotDesired
                    yWaypoint = yRobotDesired

                    previousReading = currentReading
                    previousBias = bias
                    optimalTimeToWaypointTimer = rospy.get_rostime()

                if plumeType == 1:
                    waypointDistance = sqrt( (xWaypoint - current_reading_full_data_gauss.x)**2 + (yWaypoint - current_reading_full_data_gauss.y)**2 )
                if plumeType == 2:
                    waypointDistance = sqrt( (xWaypoint - current_reading_full_data_gaden.local_x)**2 + (yWaypoint - current_reading_full_data_gaden.local_y)**2 )

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
