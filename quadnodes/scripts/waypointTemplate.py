#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.srv import CommandBool #include <mavros_msgs/CommandBool.h>
from mavros_msgs.srv import SetMode #include <mavros_msgs/SetMode.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from math import sqrt

##################
# Global variables
##################

xWaypointList = [0,5]
yWaypointList = [0,10]
zWaypointList = [2,2]

global current_state
global current_pose
current_pose = PoseStamped()
current_state = State()

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

##################
# Main
##################

def main():
    rospy.init_node('zScanNoYaw')

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("true_position", PoseStamped, pose_cb)

    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=100)

    service_timeout = 30

    rospy.wait_for_service('mavros/cmd/arming', service_timeout)
    rospy.wait_for_service("mavros/set_mode", service_timeout)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    xyzWaypointIndex = 0
    kp = 1;
    waypointRadius = 0.25;
    stayTime = 3;
    xyzError = [0, 0, 0]
    yawError = 0
    justHitWaypoint = False;
    velocityCaps = [1,1,1]
    maxSpeed = 3;

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

    last_request = rospy.get_rostime()

    while not rospy.is_shutdown():
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

        if (current_state.armed):
            xyzError[0] = xWaypointList[xyzWaypointIndex] - current_pose.pose.position.x
            xyzError[1] = yWaypointList[xyzWaypointIndex] - current_pose.pose.position.y
            xyzError[2] = zWaypointList[xyzWaypointIndex] - current_pose.pose.position.z

            withinWaypoint = sqrt(pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2))

            if(withinWaypoint <= waypointRadius):
                if( not justHitWaypoint):
                    waypointStartTime = rospy.get_rostime()
                    justHitWaypoint = True;
                if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)):
                    xyzWaypointIndex +=1
                    print("=======================")
                    print("")
                    print("Moving to next waypoint")
                    print("")
                    if xyzWaypointIndex == len(xWaypointList):
                        xyzWaypointIndex = 0
                    justHitWaypoint = False
        else:
            justHitWaypoint = False


        denom = sqrt( pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2)) # only compute denominator once per loop
        if denom == 0:
            velocityCaps[0] = 1
            velocityCaps[1] = 1
            velocityCaps[2] = 1
        else:
            velocityCaps[0] = abs((xyzError[0]/denom) *maxSpeed)
            velocityCaps[1] = abs((xyzError[1]/denom) *maxSpeed)
            velocityCaps[2] = abs((xyzError[2]/denom) *maxSpeed)

        DesiredVel.twist.linear.x = capVel(kp * xyzError[0],-velocityCaps[0],velocityCaps[0])
        DesiredVel.twist.linear.y = capVel(kp * xyzError[1],-velocityCaps[1],velocityCaps[1])
        DesiredVel.twist.linear.z = capVel(kp * xyzError[2],-velocityCaps[2],velocityCaps[2])

        local_vel_pub.publish(DesiredVel);

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
