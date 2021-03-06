#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from discritizationPF import DiscretizeMap, FindBestGoal, DiscretizeRobotPose
from math import cos, sin, pi, acos, sqrt, exp
import numpy as np
import MP_project_gs

from mavros_msgs.srv import CommandBool #include <mavros_msgs/CommandBool.h>
from mavros_msgs.srv import SetMode #include <mavros_msgs/SetMode.h>

from geometry_msgs.msg import PoseStamped #include <geometry_msgs/PoseStamped.h>
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from particle_filter.msg import particles
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

##################
# Global variables
##################

global current_state
global current_pose
global particlesEstimation
current_pose = PoseStamped()
current_state = State()
particlesEstimation = particles()
state_cb_flag = False;
pose_cb_flag = False;

##################
# Functions
##################

# This code finds the path for graphSearch
def run_IG(start,goal,map_path,actions=MP_project_gs._ACTIONS_2):
    g = MP_project_gs.GridMap(start, goal, map_path)
    # g.display_map()
    res = MP_project_gs.IG_search(g.init_pos, g.transition, g.is_goal, actions)
    pathIndex = res[0][0]
    # print("Start : ", start)
    # print("Goal : ", goal)
    # print("pathIndex : ", pathIndex)
    # print()
    # print("---------------------")
    # print(res)
    # g.display_map(res[0][0],res[1])
    # g.display_map(path)
    return pathIndex

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

def particles_cb(particles_msg):
    global particlesEstimation
    particlesEstimation = particles_msg

##################
# Main
##################

def main():
    rospy.init_node('GraphSearch')

    #We instantiate a publisher to publish the commanded local position and the appropriate clients to request arming and mode change
    rospy.Subscriber("mavros/state", State, state_cb)
    rospy.Subscriber("true_position", PoseStamped, pose_cb)
    rospy.Subscriber("particles", particles, particles_cb) # Sub to particle filter

    global_waypoint_pub = rospy.Publisher('desired_waypoint', PoseStamped, queue_size=100)

    service_timeout = 30

    rospy.wait_for_service('mavros/cmd/arming', service_timeout)
    rospy.wait_for_service("mavros/set_mode", service_timeout)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    maxVelocity        = rospy.get_param("GraphSearch/maxVelocity")     #  m/s
    zHeight            = rospy.get_param("GraphSearch/zHeight")         #  m
    waypointRadius     = rospy.get_param("GraphSearch/waypointRadius")  #  m
    stayTime           = rospy.get_param("GraphSearch/stayTime")        #  seconds
    rowSteps           = rospy.get_param("rowSteps")
    colSteps           = rospy.get_param("colSteps")
    xMinMap            = rospy.get_param("xMinMap")
    yMinMap            = rospy.get_param("yMinMap")
    xMaxMap            = rospy.get_param("xMaxMap")
    yMaxMap            = rospy.get_param("yMaxMap")

    xRange = [xMinMap, xMaxMap]
    yRange = [yMinMap, yMaxMap]

    _X = 0
    _Y = 1

    # waypoint parameters
    xyzError = [0, 0, 0]
    justHitWaypoint = False
    firstWaypointFlag = False
    path = []
    xyzWaypointIndex = 0
    optimalTimeToWaypoint = 100

    DesiredWaypoint = PoseStamped()

    waypointStartTime = rospy.get_rostime()
    optimalTimeToWaypointTimer = rospy.get_rostime()

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

    xyPoseRobot = [current_pose.pose.position.x, current_pose.pose.position.y]

    while not rospy.is_shutdown():

        # print(path)
        xyzError[0] = xWaypoint - current_pose.pose.position.x
        xyzError[1] = yWaypoint - current_pose.pose.position.y
        xyzError[2] = zWaypoint - current_pose.pose.position.z

        withinWaypoint = sqrt(pow(xyzError[0],2) + pow(xyzError[1],2) + pow(xyzError[2],2))

        if(withinWaypoint <= waypointRadius):
            if( not justHitWaypoint):
                waypointStartTime = rospy.get_rostime()
                justHitWaypoint = True;
            if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)):
                if (xyzWaypointIndex >= len(path)): #generate new waypoints
                    xyzWaypointIndex = 0
                    particleGraph,xBins,yBins,patriclesArray = DiscretizeMap(particlesEstimation.X, particlesEstimation.Y, colSteps, rowSteps, xRange, yRange)
                    xBestGoal,yBestGoal,xBestGoalIndex,yBestGoalIndex = FindBestGoal(particleGraph, patriclesArray, xyPoseRobot, xBins, yBins)#Find area of highest info

                    if type(xBestGoalIndex) != np.int64:
                        BestGoal = (xBestGoalIndex[0],yBestGoalIndex[0])
                    else:
                        BestGoal = (xBestGoalIndex,yBestGoalIndex)

                    xBestGoal = np.array([xBestGoal])
                    yBestGoal = np.array([yBestGoal])

                    # start = tuple((9-(int((xyPoseRobot[1]-5)/rowSteps)),int((xyPoseRobot[0]-1)/2)))
                    _, xIndex, yIndex = DiscretizeRobotPose([current_pose.pose.position.x, current_pose.pose.position.y], xBins, yBins)
                    start = (yIndex, xIndex)

                    if start == BestGoal:
                        BestGoal = (np.random.randint(rowSteps),np.random.randint(colSteps))


                    pathIndex = run_IG(start,BestGoal,patriclesArray,actions=MP_project_gs._ACTIONS_2)
                    path = []
                    for pathIndexCurrent in range(len(pathIndex)):
                        path.append((xBins[pathIndex[pathIndexCurrent][1]],yBins[pathIndex[pathIndexCurrent][0]]))
                    # path = run_IG(start,BestGoal,patriclesArray,actions=MP_project_gs._ACTIONS_2)

                    xWaypoint = path[xyzWaypointIndex][_X]
                    yWaypoint = path[xyzWaypointIndex][_Y]
                    optimalTimeToWaypointTimer = rospy.get_rostime()

                else: # move to next waypoint
                    xWaypoint = path[xyzWaypointIndex][_X]
                    yWaypoint = path[xyzWaypointIndex][_Y]
                    optimalTimeToWaypointTimer = rospy.get_rostime()

                # print("")
                # print("Moving to next waypoint")
                # print(xWaypoint, yWaypoint)
                # print("")
                # print("=======================")
                waypointDistance = sqrt( (xWaypoint - current_pose.pose.position.x)**2 + (yWaypoint - current_pose.pose.position.y)**2 )
                optimalTimeToWaypoint = waypointDistance/maxVelocity

                print("")
                print("waypointDistance      : ", waypointDistance)
                print("optimalTimeToWaypoint : ", optimalTimeToWaypoint)
                print("")
                print("=======================")

                xyzWaypointIndex +=1
                justHitWaypoint = False

        elif(rospy.get_rostime() - optimalTimeToWaypointTimer >= rospy.Duration(optimalTimeToWaypoint*1.5)):
            xyzWaypointIndex +=1
            if (xyzWaypointIndex >= len(path)): #generate new waypoints
                xyzWaypointIndex = 0
                particleGraph,xBins,yBins,patriclesArray = DiscretizeMap(particlesEstimation.X, particlesEstimation.Y, colSteps, rowSteps, xRange, yRange)
                xBestGoal,yBestGoal,xBestGoalIndex,yBestGoalIndex = FindBestGoal(particleGraph, patriclesArray, xyPoseRobot, xBins, yBins)#Find area of highest info

                if type(xBestGoalIndex) != np.int64:
                    BestGoal = (xBestGoalIndex[0],yBestGoalIndex[0])
                else:
                    BestGoal = (xBestGoalIndex,yBestGoalIndex)

                xBestGoal = np.array([xBestGoal])
                yBestGoal = np.array([yBestGoal])

                # start = tuple((9-(int((xyPoseRobot[1]-5)/rowSteps)),int((xyPoseRobot[0]-1)/2)))
                _, xIndex, yIndex = DiscretizeRobotPose([current_pose.pose.position.x, current_pose.pose.position.y], xBins, yBins)
                start = (yIndex, xIndex)

                if start == BestGoal:
                    BestGoal = (np.random.randint(rowSteps),np.random.randint(colSteps))


                pathIndex = run_IG(start,BestGoal,patriclesArray,actions=MP_project_gs._ACTIONS_2)
                path = []
                for pathIndexCurrent in range(len(pathIndex)):
                    path.append((xBins[pathIndex[pathIndexCurrent][1]],yBins[pathIndex[pathIndexCurrent][0]]))
                # path = run_IG(start,BestGoal,patriclesArray,actions=MP_project_gs._ACTIONS_2)

                xWaypoint = path[xyzWaypointIndex][_X]
                yWaypoint = path[xyzWaypointIndex][_Y]
                optimalTimeToWaypointTimer = rospy.get_rostime()

            else: # move to next waypoint
                xWaypoint = path[xyzWaypointIndex][_X]
                yWaypoint = path[xyzWaypointIndex][_Y]
                optimalTimeToWaypointTimer = rospy.get_rostime()

            # print("")
            # print("Could not reach waypoint moving to next waypoint")
            # print(xWaypoint, yWaypoint)
            # print("")
            # print("=======================")
            waypointDistance = sqrt( (xWaypoint - current_pose.pose.position.x)**2 + (yWaypoint - current_pose.pose.position.y)**2 )
            optimalTimeToWaypoint = waypointDistance/maxVelocity

            print("")
            print("Could not reach waypoint moving to next waypoint")
            print("waypointDistance      : ", waypointDistance)
            print("optimalTimeToWaypoint : ", optimalTimeToWaypoint)
            print("")
            print("=======================")
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
