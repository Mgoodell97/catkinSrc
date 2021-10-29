#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from mavros_msgs.msg import State #include <mavros_msgs/State.h>
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import tf_conversions
import tf
import tf2_ros
from math import sqrt
import numpy as np
##################
# Global variables
##################
global Robot_to_sensor1
global Robot_1
##################
# Functions
##################

def rasterScanGen(xRange, xNumSteps, yRange, yNumSteps):
    xArray = np.linspace(xRange[0], xRange[1], xNumSteps, endpoint=True)
    yArray = np.linspace(yRange[0], yRange[1], yNumSteps, endpoint=True)


    xScan = []
    yScan = []

    for i, yi in enumerate(yArray):
        xScan.append(xArray[::(-1)**i]) # reverse when i is odd
        yScan.append(np.ones_like(xArray) * yi)

    # squeeze lists together to vectors
    xScan = np.concatenate(xScan)
    yScan = np.concatenate(yScan)

    desiredWaypoints = np.array([xScan, yScan]).T

    return desiredWaypoints

def rasterScanGenFlipedXY(xRange, xNumSteps, yRange, yNumSteps):
    xArray = np.linspace(xRange[0], xRange[1], xNumSteps, endpoint=True)
    yArray = np.linspace(yRange[0], yRange[1], yNumSteps, endpoint=True)


    xScan = []
    yScan = []

    for i, yi in enumerate(yArray):
        xScan.append(xArray[::(-1)**i]) # reverse when i is odd
        yScan.append(np.ones_like(xArray) * yi)

    # squeeze lists together to vectors
    xScan = np.concatenate(xScan)
    yScan = np.concatenate(yScan)

    desiredWaypoints = np.array([yScan, xScan]).T

    return desiredWaypoints

##################
# Callbacks
##################

##################
# Main
##################

def main():
    rospy.init_node('fakeRobotPoseRaster_statictf')
    br = tf2_ros.TransformBroadcaster()
    static_tf = geometry_msgs.msg.TransformStamped()
    DesiredWaypoint = PoseStamped()

    xmin      = rospy.get_param("fakeRobotPoseRaster/xmin")
    xmax      = rospy.get_param("fakeRobotPoseRaster/xmax")
    Nx        = rospy.get_param("fakeRobotPoseRaster/Nx")
    ymin      = rospy.get_param("fakeRobotPoseRaster/ymin")
    ymax      = rospy.get_param("fakeRobotPoseRaster/ymax")
    Ny        = rospy.get_param("fakeRobotPoseRaster/Ny")
    stayTime  = rospy.get_param("fakeRobotPoseRaster/stayTime")
    flipXY   = rospy.get_param("fakeRobotPoseRaster/flipXY")
    RobotID   = rospy.get_param("fakeRobotPoseRaster/RobotID")
    startTest = rospy.get_param("/startTest")

    fullStringName = "/mocap_node/Robot_" + str(int(RobotID)) + "/pose"

    global_waypoint_pub = rospy.Publisher(fullStringName, PoseStamped, queue_size=1)

    waypointIndex = 0
    if flipXY:
        desiredWaypointsList = rasterScanGenFlipedXY([xmin, xmax], Nx, [ymin, ymax], Ny)
    else:
        desiredWaypointsList = rasterScanGen([xmin, xmax], Nx, [ymin, ymax], Ny)

    rate = rospy.Rate(100)

    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id = "map_gaden"
    static_tf.child_frame_id = "Robot_1/base_link"
    DesiredWaypoint.header.frame_id = "map_gaden"
    DesiredWaypoint.pose.position.x = desiredWaypointsList[waypointIndex,0]
    DesiredWaypoint.pose.position.y = desiredWaypointsList[waypointIndex,1]
    DesiredWaypoint.pose.position.z = 0.2
    static_tf.transform.translation.x = desiredWaypointsList[waypointIndex,0]
    static_tf.transform.translation.y = desiredWaypointsList[waypointIndex,1]
    static_tf.transform.translation.z = 0.2
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    static_tf.transform.rotation.x = q[0]
    static_tf.transform.rotation.y = q[1]
    static_tf.transform.rotation.z = q[2]
    static_tf.transform.rotation.w = q[3]
    DesiredWaypoint.pose.orientation.x = q[0]
    DesiredWaypoint.pose.orientation.y = q[1]
    DesiredWaypoint.pose.orientation.z = q[2]
    DesiredWaypoint.pose.orientation.w = q[3]


    global_waypoint_pub.publish(DesiredWaypoint);

    while ((not rospy.is_shutdown() and not startTest) ):
        rate.sleep()
        global_waypoint_pub.publish(DesiredWaypoint);
        br.sendTransform(static_tf)
        startTest = rospy.get_param("/startTest");
        if startTest:
            break

    waypointStartTime = rospy.get_rostime()
    last_request = rospy.get_rostime()

    while not rospy.is_shutdown():
        if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)):
            waypointIndex +=1
            waypointStartTime = rospy.get_rostime()

        if waypointIndex == len(desiredWaypointsList):
            waypointIndex = 0
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = "map_gaden"
        static_tf.child_frame_id = "Robot_1/base_link"
        DesiredWaypoint.header.frame_id = "map_gaden"
        DesiredWaypoint.pose.position.x = desiredWaypointsList[waypointIndex,0]
        DesiredWaypoint.pose.position.y = desiredWaypointsList[waypointIndex,1]
        DesiredWaypoint.pose.position.z = 0.2
        static_tf.transform.translation.x = desiredWaypointsList[waypointIndex,0]
        static_tf.transform.translation.y = desiredWaypointsList[waypointIndex,1]
        static_tf.transform.translation.z = 0.2
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        static_tf.transform.rotation.x = q[0]
        static_tf.transform.rotation.y = q[1]
        static_tf.transform.rotation.z = q[2]
        static_tf.transform.rotation.w = q[3]
        DesiredWaypoint.pose.orientation.x = q[0]
        DesiredWaypoint.pose.orientation.y = q[1]
        DesiredWaypoint.pose.orientation.z = q[2]
        DesiredWaypoint.pose.orientation.w = q[3]
        # z is always 0
        global_waypoint_pub.publish(DesiredWaypoint);
        br.sendTransform(static_tf)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
