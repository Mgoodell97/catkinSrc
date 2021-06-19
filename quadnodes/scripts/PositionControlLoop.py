#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from math import sqrt, atan2, sin, cos
from numpy import sign

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

from mavros_msgs.srv import CommandBool #include <mavros_msgs/CommandBool.h>
from mavros_msgs.srv import SetMode #include <mavros_msgs/SetMode.h>

##################
# Global variables
##################

global current_state
global current_pose
global desired_waypoint
global startPositionController
current_pose = PoseStamped()
desired_waypoint = PoseStamped()
current_state = State()
startPositionController = Bool()

state_cb_flag = False;
pose_cb_flag = False;
waypoint_cb_flag = False;

# This is for obstical avoidance id the position controller is on UAV1 it does not use UAV_avoid_1 it uses current_pose
# It uses UAV_avoid_1 and UAV_avoid_2 for obstical avoidance
global UAV_avoid_1
global UAV_avoid_2
UAV_avoid_1 = PoseStamped()
UAV_avoid_2 = PoseStamped()

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

def waypoint_cb(waypointMsg):
    global desired_waypoint
    global waypoint_cb_flag
    desired_waypoint = waypointMsg
    waypoint_cb_flag = True

def start_position_controller_cb(startPositionControllerMsg):
    global startPositionController
    startPositionController = startPositionControllerMsg

# For obstical avoidance
def UAV_avoid_1_cb(UAV_avoid_1_msg):
    global UAV_avoid_1
    UAV_avoid_1 = UAV_avoid_1_msg

def UAV_avoid_2_cb(UAV_avoid_2_msg):
    global UAV_avoid_2
    UAV_avoid_2 = UAV_avoid_2_msg

##################
# Main
##################

def main():
    rospy.init_node('PositionControlLoop')

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("true_position", PoseStamped, pose_cb)
    rospy.Subscriber("desired_waypoint", PoseStamped, waypoint_cb)
    rospy.Subscriber("/startTopic", Bool, start_position_controller_cb)

    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=100)

    service_timeout = 60 # seconds

    rospy.wait_for_service('mavros/cmd/arming', service_timeout)
    rospy.wait_for_service("mavros/set_mode", service_timeout)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    maxVelocity          = rospy.get_param("PositionControlLoop/maxVelocity")          #  m/s
    enableObsAvoid       = rospy.get_param("PositionControlLoop/enableObsAvoid")       #  [-]
    potentialFeild       = rospy.get_param("PositionControlLoop/potentialFeild")       #  m
    emergancyFeild       = rospy.get_param("PositionControlLoop/emergancyFeild")       #  m
    B                    = rospy.get_param("PositionControlLoop/potentialFeildScalor") #
    quadID               = rospy.get_param("PositionControlLoop/quadID")               #  [-]
    subToUAV1Pose        = rospy.get_param("PositionControlLoop/subToUAV1Pose")        #  [-]
    subToUAV2Pose        = rospy.get_param("PositionControlLoop/subToUAV2Pose")        #  [-]
    subToUAV3Pose        = rospy.get_param("PositionControlLoop/subToUAV3Pose")        #  [-]

    if quadID == 1:
        if subToUAV2Pose:
            rospy.Subscriber("/UAV2/true_position", PoseStamped, UAV_avoid_1_cb)
            avoidQuad1 = True
        else:
            avoidQuad1 = False
        if subToUAV3Pose:
            rospy.Subscriber("/UAV3/true_position", PoseStamped, UAV_avoid_2_cb)
            avoidQuad2 = True
        else:
            avoidQuad2 = False
    elif quadID == 2:
        if subToUAV1Pose:
            rospy.Subscriber("/UAV1/true_position", PoseStamped, UAV_avoid_1_cb)
            avoidQuad1 = True
        else:
            avoidQuad1 = False
        if subToUAV3Pose:
            rospy.Subscriber("/UAV3/true_position", PoseStamped, UAV_avoid_2_cb)
            avoidQuad2 = True
        else:
            avoidQuad2 = False
    elif quadID == 3:
        if subToUAV1Pose:
            rospy.Subscriber("/UAV1/true_position", PoseStamped, UAV_avoid_1_cb)
            avoidQuad1 = True
        else:
            avoidQuad1 = False
        if subToUAV2Pose:
            rospy.Subscriber("/UAV2/true_position", PoseStamped, UAV_avoid_2_cb)
            avoidQuad2 = True
        else:
            avoidQuad2 = False

    DesiredVelWaypoint = TwistStamped()
    VelObs1 = TwistStamped()
    VelObs2 = TwistStamped()

    # waypoint parameters
    kp = 1
    zHeight = 2
    xyzError = [0, 0, 0]
    velocityCaps = [1,1,1]

    rate = rospy.Rate(50)
    #Before publishing anything, we wait for the connection to be established between MAVROS and the autopilot.
    while ((not rospy.is_shutdown() and current_state.connected) or not state_cb_flag or not pose_cb_flag or not waypoint_cb_flag or not startPositionController.data):
        rate.sleep()
        if state_cb_flag and pose_cb_flag and startPositionController.data:
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

        try:
            xyzError[0] = desired_waypoint.pose.position.x - current_pose.pose.position.x
            xyzError[1] = desired_waypoint.pose.position.y - current_pose.pose.position.y
            xyzError[2] = desired_waypoint.pose.position.z - current_pose.pose.position.z
        except:
            xyzError[0] = 0
            xyzError[1] = 0
            xyzError[2] = 0

        denom = sqrt( pow(xyzError[0],2) + pow(xyzError[1],2) ) # only compute denominator once per loop

        if denom == 0:
            velocityCaps[0] = 1
            velocityCaps[1] = 1
        else:
            velocityCaps[0] = abs((xyzError[0]/denom) *maxVelocity)
            velocityCaps[1] = abs((xyzError[1]/denom) *maxVelocity)


        DesiredVelWaypoint.twist.linear.x = capVel(kp * xyzError[0],-velocityCaps[0],velocityCaps[0])
        DesiredVelWaypoint.twist.linear.y = capVel(kp * xyzError[1],-velocityCaps[1],velocityCaps[1])
        DesiredVelWaypoint.twist.linear.z = capVel(kp * xyzError[2],-maxVelocity,maxVelocity)


        if avoidQuad1 and enableObsAvoid:
            d = sqrt( (UAV_avoid_1.pose.position.x - current_pose.pose.position.x)**2 + (UAV_avoid_1.pose.position.y - current_pose.pose.position.y)**2 )
            theta = atan2( (UAV_avoid_1.pose.position.y - current_pose.pose.position.y) , (UAV_avoid_1.pose.position.x - current_pose.pose.position.x) )
            if d < emergancyFeild:
                VelObs1.twist.linear.x = -sign(cos(theta))*1.75
                VelObs1.twist.linear.y = -sign(sin(theta))*1.75
            elif (emergancyFeild <= d <= (emergancyFeild + potentialFeild)):
                VelObs1.twist.linear.x = -B * (potentialFeild + emergancyFeild - d) * cos(theta)
                VelObs1.twist.linear.y = -B * (potentialFeild + emergancyFeild - d) * sin(theta)
            else:
                VelObs1.twist.linear.x = 0
                VelObs1.twist.linear.x = 0

        if avoidQuad2 and enableObsAvoid:
            d = sqrt( (UAV_avoid_2.pose.position.x - current_pose.pose.position.x)**2 + (UAV_avoid_2.pose.position.y - current_pose.pose.position.y)**2 )
            theta = atan2( (UAV_avoid_2.pose.position.y - current_pose.pose.position.y) , (UAV_avoid_2.pose.position.x - current_pose.pose.position.x) )
            if d < emergancyFeild:
                VelObs2.twist.linear.x = -sign(cos(theta))*1.75
                VelObs2.twist.linear.y = -sign(sin(theta))*1.75
            elif (emergancyFeild <= d <= (emergancyFeild + potentialFeild)):
                VelObs2.twist.linear.x = -B * (potentialFeild + emergancyFeild - d) * cos(theta)
                VelObs2.twist.linear.y = -B * (potentialFeild + emergancyFeild - d) * sin(theta)
            else:
                VelObs2.twist.linear.x = 0
                VelObs2.twist.linear.x = 0


        DesiredVel.twist.linear.x = DesiredVelWaypoint.twist.linear.x + VelObs1.twist.linear.x + VelObs2.twist.linear.x
        DesiredVel.twist.linear.y = DesiredVelWaypoint.twist.linear.y + VelObs1.twist.linear.y + VelObs2.twist.linear.y
        DesiredVel.twist.linear.z = DesiredVelWaypoint.twist.linear.z + VelObs1.twist.linear.z + VelObs2.twist.linear.z

        DesiredVel.twist.linear.x = capVel(DesiredVel.twist.linear.x,-maxVelocity,maxVelocity)
        DesiredVel.twist.linear.y = capVel(DesiredVel.twist.linear.y,-maxVelocity,maxVelocity)

        local_vel_pub.publish(DesiredVel);

        # if quadID == 1:
        #     if subToUAV2Pose:
        #         print("I am quad", quadID, "avoid UAV pose x: ", UAV_avoid_1.pose.position.x, " y: ", UAV_avoid_1.pose.position.x)
        #
        # if quadID == 2:
        #     if subToUAV1Pose:
        #         print("I am quad", quadID, "avoid UAV pose x: ", UAV_avoid_1.pose.position.x, " y: ", UAV_avoid_1.pose.position.x)


        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
