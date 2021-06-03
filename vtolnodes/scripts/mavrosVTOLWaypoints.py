#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from math import sqrt, atan2, sin, cos
from numpy import sign

from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from nav_msgs.msg import Odometry

from mavros_msgs.srv import CommandBool           #include <mavros_msgs/CommandBool.h>
from mavros_msgs.srv import SetMode               #include <mavros_msgs/SetMode.h>
from mavros_msgs.srv import CommandVtolTransition #include <mavros_msgs/CommandVtolTransition.h>
from std_msgs.msg import Header

##################
# Global variables
##################

global current_state
global currentPose
currentPose = Odometry()
current_state = State()

state_cb_flag = False;
pose_cb_flag = False;
spawnBiasX = 0
spawnBiasY = 0
spawnBiasZ = 0

# This is for obstical avoidance id the position controller is on UAV1 it does not use UAV_avoid_1 it uses currentPose
# It uses UAV_avoid_1 and UAV_avoid_2 for obstical avoidance
global UAV_avoid_1
global UAV_avoid_2
UAV_avoid_1 = PoseStamped()
UAV_avoid_2 = PoseStamped()

##################
# Functions
##################

##################
# Callbacks
##################

def pose_cb(poseMsg):
    global currentPose
    global pose_cb_flag
    currentPose = poseMsg
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
    rospy.init_node('PositionControlLoop')

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("mavros/global_position/local", Odometry, pose_cb)

    local_vel_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=100)

    service_timeout = 60 # seconds

    rospy.wait_for_service('mavros/cmd/arming', service_timeout)
    rospy.wait_for_service("mavros/set_mode", service_timeout)
    rospy.wait_for_service("mavros/cmd/vtol_transition", service_timeout)
    arming_client          = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client        = rospy.ServiceProxy("mavros/set_mode", SetMode)
    vtol_transition_client = rospy.ServiceProxy("mavros/cmd/vtol_transition", CommandVtolTransition)

    global spawnBiasX
    global spawnBiasY
    global spawnBiasZ

    xWaypointList      = rospy.get_param("mavrosVTOLWaypoints/xWaypointList")   #  m
    yWaypointList      = rospy.get_param("mavrosVTOLWaypoints/yWaypointList")   #  m
    zWaypointList      = rospy.get_param("mavrosVTOLWaypoints/zWaypointList")   #  m
    waypointIndex      = rospy.get_param("mavrosVTOLWaypoints/waypointIndex")   #  [-]
    waypointRadius     = rospy.get_param("mavrosVTOLWaypoints/waypointRadius")  #  m
    spawnBiasX         = rospy.get_param("mavrosVTOLWaypoints/spawnBiasX")      #  m
    spawnBiasY         = rospy.get_param("mavrosVTOLWaypoints/spawnBiasY")      #  m
    spawnBiasZ         = rospy.get_param("mavrosVTOLWaypoints/spawnBiasZ")      #  m

    desiredWaypoint = PoseStamped()

    # waypoint parameters
    xyzError = [0, 0, 0]
    fixedWingTransitionFlag = False

    rate = rospy.Rate(50)
    #Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot.
    while ((not rospy.is_shutdown() and current_state.connected) or not state_cb_flag or not pose_cb_flag):
        rate.sleep()
        if state_cb_flag and pose_cb_flag:
            break

    desiredWaypoint = PoseStamped()
    desiredWaypoint.pose.position.x = xWaypointList[waypointIndex] - spawnBiasX
    desiredWaypoint.pose.position.y = yWaypointList[waypointIndex] - spawnBiasY
    desiredWaypoint.pose.position.z = zWaypointList[waypointIndex] - spawnBiasZ


    #send a few setpoints before starting
    #Before entering Offboard mode, you must have already started streaming setpoints. Otherwise the mode switch will be rejected. Here, 100 was chosen as an arbitrary amount.
    for i in range(100, 0, -1):
        local_vel_pub.publish(desiredWaypoint)
        rate.sleep()

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


        xyzError[0] = desiredWaypoint.pose.position.x  - currentPose.pose.pose.position.x
        xyzError[1] = desiredWaypoint.pose.position.y  - currentPose.pose.pose.position.y
        xyzError[2] = desiredWaypoint.pose.position.z  - currentPose.pose.pose.position.z

        withinWaypoint = sqrt(pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2))

        if(withinWaypoint <= waypointRadius):
            waypointIndex += 1
            if not fixedWingTransitionFlag:
                CommandVtolTransitionState = Header()
                modeResponse = vtol_transition_client(CommandVtolTransitionState,4)
                print("Vtol response : ", modeResponse)
                fixedWingTransitionFlag = True

            if waypointIndex == len(xWaypointList):
                waypointIndex = 0

            print
            print("Moving to waypoint ", waypointIndex+1)
            print
            print("====================")

        desiredWaypoint.pose.position.x = xWaypointList[waypointIndex] - spawnBiasX
        desiredWaypoint.pose.position.y = yWaypointList[waypointIndex] - spawnBiasY
        desiredWaypoint.pose.position.z = zWaypointList[waypointIndex] - spawnBiasZ

        local_vel_pub.publish(desiredWaypoint);

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
