#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
import tf_conversions
import tf
import tf2_ros
import numpy as np

# Messages
from geometry_msgs.msg import PoseStamped, TransformStamped
from olfaction_msgs.msg import gas_sensor

from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles

##################
# Global variables
##################

global chem_reading
global current_state
global sensorMsg_cb_flag

plumeMsg = estimatedGaussian()

robotSensorReadingPoseGaden = gas_sensor() # chem reading in gauss env

sensorMsg_cb_flag = False;

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

def GADENSensorAndPose_cb(GADENMsg):
    global robotSensorReadingPoseGaden
    global sensorMsg_cb_flag
    robotSensorReadingPoseGaden = GADENMsg
    sensorMsg_cb_flag = True

##################
# Main
##################

def main():

    rospy.init_node('rasterPF')

    # Create meassages
    br = tf2_ros.TransformBroadcaster()
    static_tf = TransformStamped()
    DesiredWaypoint = PoseStamped()

    # Get raster parameters
    xmin      = rospy.get_param("~xmin")
    xmax      = rospy.get_param("~xmax")
    Nx        = rospy.get_param("~Nx")
    ymin      = rospy.get_param("~ymin")
    ymax      = rospy.get_param("~ymax")
    Ny        = rospy.get_param("~Ny")
    stayTime  = rospy.get_param("~stayTime")
    flipXY   = rospy.get_param("~flipXY")
    RobotID   = rospy.get_param("~RobotID")
    printTimeSteps   = rospy.get_param("~printTimeSteps")
    startTest = rospy.get_param("/startTest")

    global sensorMsg_cb_flag

    print(Nx,Ny)

    # Set node rate
    rate = rospy.Rate(50)

    # Spoof robot pose
    fullStringName = "/mocap_node/Robot_" + str(int(RobotID)) + "/pose"

    # Create publishers
    global_waypoint_pub = rospy.Publisher(fullStringName, PoseStamped, queue_size=1)
    particlesPublisher = rospy.Publisher('particles', particles, queue_size=1)
    estimatedGaussianPublisher = rospy.Publisher("estimatedGaussian", estimatedGaussian, queue_size=10)

    # Create subcribers
    rospy.Subscriber("/Robot_1/Sensor_reading", gas_sensor, GADENSensorAndPose_cb)

    waypointIndex = 0
    if flipXY:
        desiredWaypointsList = rasterScanGenFlipedXY([xmin, xmax], Nx, [ymin, ymax], Ny)
    else:
        desiredWaypointsList = rasterScanGen([xmin, xmax], Nx, [ymin, ymax], Ny)


    print(len(desiredWaypointsList))

    # Sensor data stuff
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


    k = 0
    while not rospy.is_shutdown():
        # Wait until new sensor reading is recieved
        while ((not rospy.is_shutdown() and not sensorMsg_cb_flag) ):
            rate.sleep()
            global_waypoint_pub.publish(DesiredWaypoint);
            br.sendTransform(static_tf)
            if sensorMsg_cb_flag:
                sensorMsg_cb_flag = False
                break




        if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)):
            k +=1

            # Get sensor reading
            z_t = robotSensorReadingPoseGaden.raw
            x_t = np.array([robotSensorReadingPoseGaden.local_x, robotSensorReadingPoseGaden.local_y, robotSensorReadingPoseGaden.local_z])

            if printTimeSteps:

                print("================================")
                print()
                print("Robot pose : ", x_t)
                print("z [ppm]    : ", z_t)
                print("k          : ", k)
                print()
                # print("Std        : ", R1_Pf.stdL2Norm)

            # Move robot to next waypoint
            waypointIndex +=1
            waypointStartTime = rospy.get_rostime()

            # Finish script when waypoint list runs out
            if waypointIndex == len(desiredWaypointsList):
                print("Simulation has finished")
                break







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
        br.sendTransform(static_tf)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
