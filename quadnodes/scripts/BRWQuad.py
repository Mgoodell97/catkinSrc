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

def capVel(currentVelocity, minVel = 3,  maxVel = 3):
    if currentVelocity > maxVel:
        return maxVel
    if currentVelocity < minVel:
        return minVel
    return currentVelocity

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

    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=100)

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
    maxVelocity        = rospy.get_param("BRWQuad/maxVelocity")     #  m/s
    GaussPlume         = rospy.get_param("BRWQuad/GaussPlume")      #  I'm not explaining this
    GadenPlume         = rospy.get_param("BRWQuad/GadenPlume")      #  I'm not explaining this

    if GaussPlume:
        rospy.Subscriber("gaussianReading", gaussian, gaussSensor_cb)
    if GadenPlume:
        rospy.Subscriber("Sensor_reading", gas_sensor, gadenSensor_cb)


    # minLim = 0
    # maxLim = 50
    biasRange = biasRange * pi/180 # converts to radians
    # stepSize = 5
    xyzWaypointIndex = 0
    kp = 1
    # waypointRadius = 0.1
    # stayTime = 0.5;
    xyzError = [0, 0, 0]
    yawError = 0
    justHitWaypoint = False
    firstWaypointFlag = False

    waypointStartTime = rospy.get_rostime()

    rate = rospy.Rate(50)
    #Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot.
    while ((not rospy.is_shutdown() and current_state.connected) or not state_cb_flag or not pose_cb_flag):
        rate.sleep()
        if state_cb_flag and pose_cb_flag:
            break

    DesiredVel = TwistStamped()
    DesiredVel.twist.linear.x = 0
    DesiredVel.twist.linear.y = 0
    DesiredVel.twist.linear.z = 0
    DesiredVel.twist.angular.z = 0

    #send a few setpoints before starting
    #Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will be rejected. Here, 100 was chosen as an arbitrary amount.
    for i in range(100, 0, -1):
        local_vel_pub.publish(DesiredVel)
        rate.sleep()

    # Start first waypoint right above the robot
    xWaypointList = current_pose.pose.position.x
    yWaypointList = current_pose.pose.position.y
    zWaypointList = zHeight

    last_request = rospy.get_rostime()

    while not rospy.is_shutdown():
        # Arming and safty checks for the robot
        if ( not current_state.mode == "OFFBOARD" and (rospy.get_rostime() - last_request) > rospy.Duration.from_sec(5.0)):
            modeResponse = set_mode_client(0,"OFFBOARD")
            if (modeResponse.mode_sent):
                rospy.loginfo("Offboard enabled")
            last_request = rospy.get_rostime()
        else:
            if (not current_state.armed and ((rospy.get_rostime() - last_request) > rospy.Duration.from_sec(5.0))):
                armResponse = arming_client(True)
                if(armResponse.success):
                    rospy.loginfo("Vehicle armed")
                last_request = rospy.get_rostime()

        # Once robot is armed start motion planning logic
        if (current_state.armed):

            xyzError[0] = xWaypointList - current_pose.pose.position.x
            xyzError[1] = yWaypointList - current_pose.pose.position.y
            xyzError[2] = zWaypointList - current_pose.pose.position.z

            withinWaypoint = sqrt(pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2))

            if(withinWaypoint <= waypointRadius):
                if( not justHitWaypoint):
                    waypointStartTime = rospy.get_rostime()
                    justHitWaypoint = True;
                if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)):
                    if not firstWaypointFlag:
                        # Get first reading
                        if GaussPlume:
                            previousReading = current_reading_full_data_gauss.ppm
                        if GadenPlume:
                            previousReading = current_reading_full_data_gaden.raw
                        xRobotDesired, yRobotDesired = moveRobot(current_pose.pose.position.x, current_pose.pose.position.y, stepSize, minLim, maxLim)
                        xWaypointList = xRobotDesired
                        yWaypointList = yRobotDesired
                        previousBias = np.arctan2((yRobotDesired-current_pose.pose.position.y),(xRobotDesired-current_pose.pose.position.x))
                        #Only move randomly once
                        firstWaypointFlag = True
                    else: # Start bias random walk
                        if GaussPlume:
                            currentReading = current_reading_full_data_gauss.ppm
                        if GadenPlume:
                            currentReading = current_reading_full_data_gaden.raw
                        xRobotDesired, yRobotDesired, slope, bias = biasedRandomWalk(current_pose.pose.position.x, current_pose.pose.position.y, previousReading, currentReading, biasRange, previousBias, stepSize, minLim, maxLim)
                        xWaypointList = xRobotDesired
                        yWaypointList = yRobotDesired

                        previousReading = currentReading
                        previousBias = bias

                    # print("")
                    # print("Moving to next waypoint")
                    # print("")
                    # print("=======================")

                    justHitWaypoint = False
        else:
            justHitWaypoint = False

        DesiredVel.twist.linear.x = capVel(kp * xyzError[0],-maxVelocity,maxVelocity)
        DesiredVel.twist.linear.y = capVel(kp * xyzError[1],-maxVelocity,maxVelocity)
        DesiredVel.twist.linear.z = capVel(kp * xyzError[2],-maxVelocity,maxVelocity)

        local_vel_pub.publish(DesiredVel);

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
