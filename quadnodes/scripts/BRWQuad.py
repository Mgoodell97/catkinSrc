#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.srv import CommandBool #include <mavros_msgs/CommandBool.h>
from mavros_msgs.srv import SetMode #include <mavros_msgs/SetMode.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from quadnodes.msg import gaussian
from olfaction_msgs.msg import gas_sensor

from math import sqrt, pi
import numpy as np

from BRW_functions import biasedRandomWalk, moveRobot

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
    rospy.init_node('BRW')

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("true_position", PoseStamped, pose_cb)

    global_waypoint_pub = rospy.Publisher('desired_waypoint', PoseStamped, queue_size=100)

    service_timeout = 30

    rospy.wait_for_service('mavros/cmd/arming', service_timeout)
    rospy.wait_for_service("mavros/set_mode", service_timeout)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    minLim             = rospy.get_param("BRWQuad/minLim")          #  m
    maxLim             = rospy.get_param("BRWQuad/maxLim")          #  m
    zHeight            = rospy.get_param("BRWQuad/zHeight")         #  m
    biasRange          = rospy.get_param("BRWQuad/biasRange")       #  degrees
    stepSize           = rospy.get_param("BRWQuad/stepSize")        #  m
    waypointRadius     = rospy.get_param("BRWQuad/waypointRadius")  #  m
    stayTime           = rospy.get_param("BRWQuad/stayTime")        #  seconds
    PlumeType          = rospy.get_param("BRWQuad/PlumeType")       #  I'm not explaining this

    if PlumeType == "gaussian":
        rospy.Subscriber("gaussianReading", gaussian, gaussSensor_cb)
    if PlumeType == "gaden":
        rospy.Subscriber("Sensor_reading", gas_sensor, gadenSensor_cb)

    biasRange = biasRange * pi/180 # converts to radians
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

    last_request = rospy.get_rostime()

    # Start first waypoint right above the robot
    xWaypoint = current_pose.pose.position.x
    yWaypoint = current_pose.pose.position.y
    zWaypoint = zHeight

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
                if not firstWaypointFlag:
                    # Get first reading
                    if PlumeType == "gaussian":
                        previousReading = current_reading_full_data_gauss.ppm
                    if PlumeType == "gaden":
                        previousReading = current_reading_full_data_gaden.raw
                    xRobotDesired, yRobotDesired = moveRobot(current_pose.pose.position.x, current_pose.pose.position.y, stepSize, minLim, maxLim)
                    xWaypoint = xRobotDesired
                    yWaypoint = yRobotDesired
                    previousBias = np.arctan2((yRobotDesired-current_pose.pose.position.y),(xRobotDesired-current_pose.pose.position.x))
                    #Only move randomly once
                    firstWaypointFlag = True
                else: # Start bias random walk
                    if PlumeType == "gaussian":
                        currentReading = current_reading_full_data_gauss.ppm
                    if PlumeType == "gaden":
                        currentReading = current_reading_full_data_gaden.raw
                    xRobotDesired, yRobotDesired, slope, bias = biasedRandomWalk(current_pose.pose.position.x, current_pose.pose.position.y, previousReading, currentReading, biasRange, previousBias, stepSize, minLim, maxLim)
                    xWaypoint = xRobotDesired
                    yWaypoint = yRobotDesired

                    previousReading = currentReading
                    previousBias = bias

                # print("")
                # print("Moving to next waypoint")
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
