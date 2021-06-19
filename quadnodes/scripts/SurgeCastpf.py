#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.srv import CommandBool #include <mavros_msgs/CommandBool.h>
from mavros_msgs.srv import SetMode #include <mavros_msgs/SetMode.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from quadnodes.msg import gaussian
from particle_filter.msg import estimatedGaussian
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
global plumeEstimation
current_pose = PoseStamped()
current_state = State()
current_reading_full_data_gauss = gaussian()
current_reading_full_data_gaden = gas_sensor()
plumeEstimation = estimatedGaussian()

state_cb_flag = False;
pose_cb_flag = False;

##################
# Functions
##################

def gaussFunc(xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc + (zFunc**2)/DzFunc))
    return con


def getReading(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    # Rotate frame

    Xw_r = np.array([xRobotDef, yRobotDef, 1])

    R = np.array([[cos(thetaFunc), -sin(thetaFunc)], [sin(thetaFunc), cos(thetaFunc)]])
    P = np.array([[xPlumeFunc, yPlumeFunc]]).T;

    bottomArray = np.array([[0, 0 , 1]]);
    Tw_plumeFrame = np.concatenate((R, P), axis=1)

    # Transfer matrix from plumeframe to w
    Tw_plumeFrame = np.concatenate((Tw_plumeFrame, bottomArray), axis=0)

    Rinv = np.linalg.inv(R)
    Pinv = np.array([np.matmul(-Rinv, np.squeeze(P))]).T

    TplumeFrame_w = np.concatenate((Rinv, Pinv), axis=1)

    # Transfer matrix from w to plumeframe
    TplumeFrame_w = np.concatenate((TplumeFrame_w, bottomArray), axis=0)

    XplumeFrame = np.matmul(TplumeFrame_w, Xw_r)

    xRobotRotated = XplumeFrame[0]
    yRobotRotated = XplumeFrame[1]


    if xRobotRotated <= 0:
        reading = 0
    else:
        reading = gaussFunc(xRobotRotated,yRobotRotated,zFunc,QFunc,vFunc,DyFunc,DzFunc)

    return reading

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

def pf_cb(pfMsg):
    global plumeEstimation
    plumeEstimation = pfMsg

##################
# Main
##################

def main():
    rospy.init_node('SurgeCast')

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("true_position", PoseStamped, pose_cb)
    rospy.Subscriber("estimatedGaussian", estimatedGaussian, pf_cb) # Sub to particle filter

    global_waypoint_pub = rospy.Publisher('desired_waypoint', PoseStamped, queue_size=100)

    service_timeout = 30

    rospy.wait_for_service('mavros/cmd/arming', service_timeout)
    rospy.wait_for_service("mavros/set_mode", service_timeout)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    minLim             = rospy.get_param("SurgeCastpf/minLim")          #  m
    maxLim             = rospy.get_param("SurgeCastpf/maxLim")          #  m
    zHeight            = rospy.get_param("SurgeCastpf/zHeight")         #  m
    threshold          = rospy.get_param("SurgeCastpf/threshold")       #  ppm
    stepSize           = rospy.get_param("SurgeCastpf/stepSize")        #  m
    waypointRadius     = rospy.get_param("SurgeCastpf/waypointRadius")  #  m
    stayTime           = rospy.get_param("SurgeCastpf/stayTime")        #  seconds

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
    xRobot, yRobot = rotateRobot_WorldToRobot(current_pose.pose.position.x, current_pose.pose.position.y, plumeEstimation.Theta + pi/2, current_pose.pose.position.x, current_pose.pose.position.y)
    robotThetaCurrent = 0

    #
    xRobotSurgeOffset = xRobot
    yRobotSurgeOffset = yRobot
    r1 = stepSize

    # For tf frames
    xRobotTf = current_pose.pose.position.x
    yRobotTf = current_pose.pose.position.y
    thetaRotation = plumeEstimation.Theta + pi/2

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
                currentReading = getReading(current_pose.pose.position.x, current_pose.pose.position.y, plumeEstimation.Theta, plumeEstimation.X, plumeEstimation.Y, plumeEstimation.Z - current_pose.pose.position.z, plumeEstimation.Q, plumeEstimation.V, plumeEstimation.Dy, plumeEstimation.Dz)

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
