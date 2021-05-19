#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from quadnodes.msg import gaussian
from olfaction_msgs.msg import gas_sensor

from math import cos, sin, pi, acos, sqrt, exp
import numpy as np

from Surge_Cast_functions import rotateRobot_WorldToRobot, rotateRobot_RobotToWorld

##################
# Global variables
##################

global current_state
global current_pose
global current_reading_full_data_gauss
global current_reading_full_data_gaden
current_pose = PoseStamped()
current_state = State()
current_reading_full_data_gauss = gaussian()
current_reading_full_data_gaden = gas_sensor()

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

def gaussSensor_cb(gaussMsg):
    global current_reading_full_data_gauss
    current_reading_full_data_gauss = gaussMsg

def gadenSensor_cb(gadenMsg):
    global current_reading_full_data_gaden
    current_reading_full_data_gaden = gadenMsg

##################
# Main
##################

def main():
    rospy.init_node('SurgeCast')

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("true_position", PoseStamped, pose_cb)

    global_waypoint_pub = rospy.Publisher('desired_waypoint', PoseStamped, queue_size=100)

    service_timeout = 30

    minLim             = rospy.get_param("SurgeCastQuad/minLim")          #  m
    maxLim             = rospy.get_param("SurgeCastQuad/maxLim")          #  m
    thetaPlume         = rospy.get_param("/thetaPlume")                   #  degrees (for surge cast direction (wind data))
    threshold          = rospy.get_param("SurgeCastQuad/threshold")       #  ppm
    zHeight            = rospy.get_param("SurgeCastQuad/zHeight")         #  m
    stepSize           = rospy.get_param("SurgeCastQuad/stepSize")        #  m
    waypointRadius     = rospy.get_param("SurgeCastQuad/waypointRadius")  #  m
    stayTime           = rospy.get_param("SurgeCastQuad/stayTime")        #  seconds
    PlumeType          = rospy.get_param("SurgeCastQuad/PlumeType")       #  I'm not explaining this

    thetaPlume = thetaPlume * pi/180

    if PlumeType == "gaussian":
        rospy.Subscriber("gaussianReading", gaussian, gaussSensor_cb)
    if PlumeType == "gaden":
        rospy.Subscriber("Sensor_reading", gas_sensor, gadenSensor_cb)

    # waypoint parameters
    xyzError = [0, 0, 0]
    justHitWaypoint = False
    firstWaypointFlag = False

    DesiredWaypoint = PoseStamped()

    waypointStartTime = rospy.get_rostime()

    rate = rospy.Rate(50)

    while ((not rospy.is_shutdown() and current_state.connected) or not state_cb_flag or not pose_cb_flag):
        rate.sleep()
        if state_cb_flag and pose_cb_flag:
            break

    # surge casting params
    xRobot, yRobot = rotateRobot_WorldToRobot(current_pose.pose.position.x, current_pose.pose.position.y, thetaPlume + pi/2, current_pose.pose.position.x, current_pose.pose.position.y)
    robotThetaCurrent = 0

    #
    xRobotSurgeOffset = xRobot
    yRobotSurgeOffset = yRobot
    r1 = stepSize

    # For tf frames
    xRobotTf = current_pose.pose.position.x
    yRobotTf = current_pose.pose.position.y
    thetaRotation = thetaPlume + pi/2

    # Start first waypoint right above the robot
    xWaypoint = current_pose.pose.position.x
    yWaypoint = current_pose.pose.position.y
    zWaypoint = zHeight

    last_request = rospy.get_rostime()

    while not rospy.is_shutdown():
        xyzError[0] = xWaypoint - current_pose.pose.position.x
        xyzError[1] = yWaypoint - current_pose.pose.position.y
        xyzError[2] = zWaypoint - current_pose.pose.position.z

        withinWaypoint = sqrt(pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2))

        if(withinWaypoint <= waypointRadius):
            if( not justHitWaypoint):
                waypointStartTime = rospy.get_rostime()
                justHitWaypoint = True;
            if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)):
                # Get current reading
                if PlumeType == "gaussian":
                    currentReading = current_reading_full_data_gauss.ppm
                if PlumeType == "gaden":
                    currentReading = current_reading_full_data_gaden.raw

                if currentReading < threshold: # below threshold do spinarony
                    r1Squared = pow(r1,2)
                    stepSizeSquared = pow(stepSize,2)
                    r2 = sqrt(r1Squared + stepSizeSquared)
                    r2Squared = pow(r2,2)

                    robotThetaCurrentNew = acos( (r1Squared + r2Squared - stepSizeSquared)/(2*r1*r2) )
                    robotThetaCurrent += robotThetaCurrentNew

                    xRobot = r2 * cos(robotThetaCurrent) + xRobotSurgeOffset
                    yRobot = r2 * sin(robotThetaCurrent) + yRobotSurgeOffset

                    r1 = r2

                else: # above threshold go towards plume
                    robotThetaCurrent = 0
                    xRobotSurgeOffset = xRobot
                    yRobotSurgeOffset = yRobot
                    r1 = stepSize

                    yRobot += stepSize

                xWaypoint, yWaypoint = rotateRobot_RobotToWorld(xRobot, yRobot, thetaRotation, xRobotTf, yRobotTf)
                # print("")
                # print("Moving to next waypoint")
                # print(xWaypoint, yWaypoint)
                # print("")
                # print("=======================")

                justHitWaypoint = False
        else:
            justHitWaypoint = False

        DesiredWaypoint.pose.position.x = xWaypoint
        DesiredWaypoint.pose.position.y = yWaypoint
        DesiredWaypoint.pose.position.z = zHeight
        global_waypoint_pub.publish(DesiredWaypoint);

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
