#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from math import sqrt

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
    rospy.init_node('waypointListNode')

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("true_position", PoseStamped, pose_cb)

    global_waypoint_pub = rospy.Publisher('desired_waypoint', PoseStamped, queue_size=100)

    service_timeout = 30

    xWaypointList      = rospy.get_param("waypointListNode/xWaypointList")   #  m
    yWaypointList      = rospy.get_param("waypointListNode/yWaypointList")   #  m
    zWaypointList      = rospy.get_param("waypointListNode/zWaypointList")   #  m
    waypointIndex      = rospy.get_param("waypointListNode/waypointIndex")   #  [-]
    waypointRadius     = rospy.get_param("waypointListNode/waypointRadius")  #  m
    stayTime           = rospy.get_param("waypointListNode/stayTime")        #  seconds


    xyzError = [0, 0, 0]
    justHitWaypoint = False;

    DesiredWaypoint = PoseStamped()

    waypointStartTime = rospy.get_rostime()

    rate = rospy.Rate(50)

    while ((not rospy.is_shutdown() and current_state.connected) or not state_cb_flag or not pose_cb_flag):
        rate.sleep()
        if state_cb_flag and pose_cb_flag:
            break

    last_request = rospy.get_rostime()

    while not rospy.is_shutdown():
        xyzError[0] = xWaypointList[waypointIndex] - current_pose.pose.position.x
        xyzError[1] = yWaypointList[waypointIndex] - current_pose.pose.position.y
        xyzError[2] = zWaypointList[waypointIndex] - current_pose.pose.position.z

        withinWaypoint = sqrt(pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2))

        if(withinWaypoint <= waypointRadius):
            if( not justHitWaypoint):
                waypointStartTime = rospy.get_rostime()
                justHitWaypoint = True;
            if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)):
                waypointIndex +=1
                print("=======================")
                print("")
                print("Moving to next waypoint")
                # for j in range(len(xWaypointListTemp)):
                #     print("Value : ", xWaypointListTemp[j], "   Type : ", type(xWaypointListTemp[j]))
                print("")

                if waypointIndex == len(xWaypointList):
                    waypointIndex = 0
                justHitWaypoint = False
        else:
            justHitWaypoint = False

        DesiredWaypoint.pose.position.x = xWaypointList[waypointIndex]
        DesiredWaypoint.pose.position.y = yWaypointList[waypointIndex]
        DesiredWaypoint.pose.position.z = zWaypointList[waypointIndex]
        global_waypoint_pub.publish(DesiredWaypoint);

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
