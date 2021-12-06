#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
import tf_conversions
import tf
import tf2_ros
import numpy as np
import os, rospkg
import json
import pickle
from math import pi

# Custom modules
from particleFilterPackage import ParticleFilter
from rasterScanGeneration import rasterScanGen, rasterScanGenYX
from GaussianSensorPackage import combinePlumesNew, GaussianMultiPlumeNBNoClass

# Messages
from geometry_msgs.msg import PoseStamped, TransformStamped
from olfaction_msgs.msg import gas_sensor

from particle_filter.msg import particles

##################
# Global variables
##################

global chem_reading
global current_state
global sensorMsg_cb_flag

robotSensorReadingPoseGaden = gas_sensor() # chem reading in gauss env

sensorMsg_cb_flag = False;

##################
# Functions
##################



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
    br                = tf2_ros.TransformBroadcaster()
    static_tf         = TransformStamped()
    DesiredWaypoint   = PoseStamped()
    estimatedGaussMsg = particles()
    particlesMsg      = particles()
    consumedPlumesMsg = particles()

    # Get raster parameters
    printTimeSteps  = rospy.get_param("~printTimeSteps")
    particle_params = rospy.get_param("rasterPF") # get params defined in launch file

    # Saves dictionary entries from launch file as variables
    for key,val in particle_params.items():
        exec(key + '=val')

    startTest = rospy.get_param("/startTest")

    global sensorMsg_cb_flag

    # Set node rate
    rate = rospy.Rate(50)

    # Spoof robot pose
    fullStringName = "/mocap_node/Robot_" + str(int(RobotID)) + "/pose"

    # Create publishers
    global_waypoint_pub        = rospy.Publisher(fullStringName,      PoseStamped, queue_size=1)
    particlesPublisher         = rospy.Publisher('particles',         particles, queue_size=1)
    estimatedGaussianPublisher = rospy.Publisher("estimatedGaussian", particles, queue_size=10)
    consumedPlumesPublisher    = rospy.Publisher("consumedPlumes",    particles, queue_size=1)

    # Create subcribers
    rospy.Subscriber("/Robot_1/Sensor_reading", gas_sensor, GADENSensorAndPose_cb)

    # =============================================================================
    # RasterScanGen
    # =============================================================================
    waypointIndex = 0
    if flipXY:
        desiredWaypointsList = rasterScanGenYX([xmin, xmax], Nx, [ymin, ymax], Ny)
    else:
        desiredWaypointsList = rasterScanGen([xmin, xmax], Nx, [ymin, ymax], Ny)

    # =============================================================================
    # PF params
    # =============================================================================

    Theta_u = windDir+Theta_range

    # =============================================================================
    # Create PF
    # =============================================================================

    X_n = (X_u - X_l) * mapPercent
    Y_n = (Y_u - Y_l) * mapPercent
    Z_n = (Z_u - Z_l) * mapPercent
    Theta_n = (Theta_u - Theta_l) * percentTheta
    Q_n = (Q_u - Q_l) * percentQ
    V_n = (V_u - V_l) * percentV
    Dy_n = (Dy_u - Dy_l) * percentDy
    Dz_n = (Dz_u - Dz_l) * percentDz

    randomStateBoundsVec = np.array([[X_l, X_u],[Y_l, Y_u],[Z_l, Z_u],[Theta_l,Theta_u],[Q_l,Q_u],[V_l,V_u],[Dy_l,Dy_u],[Dz_l, Dz_u]])
    noiseVec = np.array([X_n,Y_n,Z_n,Theta_n,Q_n,V_n,Dy_n,Dz_n])

    R1_Pf = ParticleFilter(randomStateBoundsVec, noiseVec, pdf_std, NumOfParticles, NumberOfParticlesToReset)

    Xfound     = np.array([], dtype=np.float32)
    Yfound     = np.array([], dtype=np.float32)
    Zfound     = np.array([], dtype=np.float32)
    Thetafound = np.array([], dtype=np.float32)
    Qfound     = np.array([], dtype=np.float32)
    Vfound     = np.array([], dtype=np.float32)
    Dyfound    = np.array([], dtype=np.float32)
    Dzfound    = np.array([], dtype=np.float32)
    Ahat = np.array([Xfound, Yfound, Zfound, Thetafound, Qfound, Vfound, Dyfound, Dzfound])
    PlotFlag   = False
    PFFlag     = False

    multiPlumeSub = GaussianMultiPlumeNBNoClass(Thetafound, Xfound, Yfound, Zfound, Qfound, Vfound, Dyfound, Dzfound)

    # =============================================================================
    # Saving data
    # =============================================================================

    kVec     = []
    xVec     = []
    A        = []
    alphaHat = []
    zVec     = []
    stdVec   = []

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

        # Once a new reading has been reieved and your at the waypoint perform estimation and move robot
        if(rospy.get_rostime() - waypointStartTime >= rospy.Duration(stayTime)):
            k +=1

            # 1. Get Robot Pose
            x_t = np.array([robotSensorReadingPoseGaden.local_x, robotSensorReadingPoseGaden.local_y, robotSensorReadingPoseGaden.local_z])

            # 2. Get sensor reading and modifiy it with found plumes
            z_t = robotSensorReadingPoseGaden.raw - multiPlumeSub.getReading(x_t[0], x_t[1], x_t[2])

            kVec.append(k)
            xVec.append(x_t)
            zVec.append(z_t)

            # 3. Measurement prediction
            gaussHatVec = R1_Pf.calculateXhatNumbaNew(z_t, x_t, Ahat)

            # 3.5 publish current gaussian estimate and particles
            estimatedGaussMsg.X           = (gaussHatVec[0],)
            estimatedGaussMsg.Y           = (gaussHatVec[1],)
            estimatedGaussMsg.Z           = (gaussHatVec[2],)
            estimatedGaussMsg.theta       = (gaussHatVec[3],)
            estimatedGaussMsg.Q           = (gaussHatVec[4],)
            estimatedGaussMsg.v           = (gaussHatVec[5],)
            estimatedGaussMsg.Dy          = (gaussHatVec[6],)
            estimatedGaussMsg.Dz          = (gaussHatVec[7],)

            particlesMsg.X       = R1_Pf.xp[:,0]
            particlesMsg.Y       = R1_Pf.xp[:,1]
            particlesMsg.Z       = R1_Pf.xp[:,2]
            particlesMsg.theta   = R1_Pf.xp[:,3]
            particlesMsg.Q       = R1_Pf.xp[:,4]
            particlesMsg.v       = R1_Pf.xp[:,5]
            particlesMsg.Dy      = R1_Pf.xp[:,6]
            particlesMsg.Dz      = R1_Pf.xp[:,7]
            particlesMsg.weights = R1_Pf.wp

            estimatedGaussianPublisher.publish(estimatedGaussMsg)
            particlesPublisher.publish(particlesMsg)

            if printTimeSteps:

                print("================================")
                print()
                print("Robot pose : ", x_t)
                print("z [ppm]    : ", z_t)
                print("k          : ", k)
                print("Std        : ", R1_Pf.stdL2Norm)
                print()

            # 4. Gain confidence and consume plume
            if confidenceThreshold >= R1_Pf.stdL2Norm:# and sensorCount>=3: # if confidence is high enough and have at least 3 measurements consume plume
                # print("Found plume")
                Xfound = np.append(Xfound, gaussHatVec[0])
                Yfound = np.append(Yfound, gaussHatVec[1])
                Zfound = np.append(Zfound, gaussHatVec[2])
                Thetafound = np.append(Thetafound, gaussHatVec[3])
                Qfound = np.append(Qfound, gaussHatVec[4])
                Vfound = np.append(Vfound, gaussHatVec[5])
                Dyfound = np.append(Dyfound, gaussHatVec[6])
                Dzfound = np.append(Dzfound, gaussHatVec[7])

                if len(Xfound) > 1:
                    plumeList = np.array([Xfound, Yfound, Zfound, Thetafound, Qfound, Vfound, Dyfound, Dzfound])

                    Xfound, Yfound, Zfound, Thetafound, Qfound, Vfound, Dyfound, Dzfound = combinePlumesNew(plumeList, combineDistanceThreshold)

                Xfound = Xfound.astype(dtype=np.float32)
                Yfound = Yfound.astype(dtype=np.float32)
                Zfound = Zfound.astype(dtype=np.float32)
                Thetafound = Thetafound.astype(dtype=np.float32)
                Qfound = Qfound.astype(dtype=np.float32)
                Vfound = Vfound.astype(dtype=np.float32)
                Dyfound = Dyfound.astype(dtype=np.float32)
                Dzfound = Dzfound.astype(dtype=np.float32)

                Ahat = np.array([Xfound, Yfound, Zfound, Thetafound, Qfound, Vfound, Dyfound, Dzfound])
                multiPlumeSub = GaussianMultiPlumeNBNoClass(Thetafound, Xfound, Yfound, Zfound, Qfound, Vfound, Dyfound, Dzfound)
                R1_Pf.resetParticles()
                R1_Pf.wp = np.ones(NumOfParticles) * 1/NumOfParticles
                # R1_Pf.updateParticlesFromPastMeasurements(z, xVec, multiPlumeSub)
                R1_Pf.updateParticlesFromPastMeasurementsNumbaNew(zVec, xVec, Ahat)

                print(Ahat)

            consumedPlumesMsg.X     = Xfound
            consumedPlumesMsg.Y     = Yfound
            consumedPlumesMsg.Z     = Zfound
            consumedPlumesMsg.theta = Thetafound
            consumedPlumesMsg.Q     = Qfound
            consumedPlumesMsg.v     = Vfound
            consumedPlumesMsg.Dy    = Dyfound
            consumedPlumesMsg.Dz    = Dzfound

            consumedPlumesPublisher.publish(consumedPlumesMsg)

            # 5. Resample (if needed)
            resampleCheck = 1/sum(R1_Pf.wp**2)
            if (resampleCheck <= NumOfParticles/2):
                R1_Pf.updateParticles(resample = True)
            else:
                R1_Pf.updateParticles(resample = False)

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
