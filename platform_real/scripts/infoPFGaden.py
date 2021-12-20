#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
import tf_conversions
import tf
from visualization_msgs.msg import Marker
import tf2_ros
import numpy as np
import os, rospkg
import json
import pickle
from math import pi
import matplotlib.pyplot as plt

# Custom modules
from scipy.special import roots_legendre
from particleFilterPackage import ParticleFilter
from InformationThearyPackage import RobotMotion, informationAtXNewNB
from GaussianSensorPackage import combinePlumesNew, getReadingMultiPlume

# Messages
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from olfaction_msgs.msg import gas_sensor

from particle_filter.msg import particles

##################
# Global variables
##################

global sensorMsg_cb_flag
global ppm_reading
global x_t

sensorMsg_cb_flag = False;
ppm_reading = 0
x_t = np.zeros(3)

##################
# Functions
##################



##################
# Callbacks
##################

def GADENSensorAndPose_cb(GADENMsg):
    global sensorMsg_cb_flag
    global ppm_reading
    global x_t
    ppm_reading = GADENMsg.raw
    x_t = np.array([GADENMsg.local_x, GADENMsg.local_y, GADENMsg.local_z])
    sensorMsg_cb_flag = True

def MPS_cb(MPS_cb_msg):
    global ppm_reading
    global x_t
    ppm_reading = MPS_cb_msg.pressure
    x_t = np.array([MPS_cb_msg.local_x, MPS_cb_msg.local_y, MPS_cb_msg.local_z])

##################
# Main
##################

def main():

    rospy.init_node('infoPF')

    # Create meassages
    br                = tf2_ros.TransformBroadcaster()
    static_tf         = TransformStamped()
    DesiredWaypoint   = PoseStamped()
    estimatedGaussMsg = particles()
    particlesMsg      = particles()
    consumedPlumesMsg = particles()

    # Get raster parameters
    printTimeSteps  = rospy.get_param("~printTimeSteps")
    particle_params = rospy.get_param("PF") # get params defined in launch file
    startTest  = rospy.get_param("/startTest")
    xPlumeLoc  = rospy.get_param("/xPlumeLoc")
    yPlumeLoc  = rospy.get_param("/yPlumeLoc")
    simNumber  = rospy.get_param("/simNumber")
    simType    = rospy.get_param("/simType")
    saveResults  = rospy.get_param("/saveResults")

    # Saves dictionary entries from launch file as variables
    for key,val in particle_params.items():
        exec(key + '=val')

    global sensorMsg_cb_flag
    global x_t

    # Set node rate
    rate = rospy.Rate(50)

    # Create publishers
    particlesPublisher         = rospy.Publisher('particles',         particles, queue_size=1)
    estimatedGaussianPublisher = rospy.Publisher("estimatedGaussian", particles, queue_size=10)
    consumedPlumesPublisher    = rospy.Publisher("consumedPlumes",    particles, queue_size=1)
    particlesRVIZ              = rospy.Publisher("particlesRVIZ",     Marker, queue_size=1)
    global_waypoint_pub        = rospy.Publisher('desiredPos',        PoseStamped, queue_size=10)

    # Create subcribers
    if simType == 2:
        rospy.Subscriber("/Robot_1/Sensor_reading", gas_sensor, GADENSensorAndPose_cb)
    elif simType == 3:
        rospy.Subscriber("mps_data", MPS, MPS_cb)

    # =============================================================================
    # Setup information based motion planner
    # =============================================================================

    xStart = 3.5
    yStart = 0.5

    R1 = RobotMotion(PoseVec = np.array([xStart,yStart,1]), VelVec = np.array([0,0,0]), MaxVel=0.375)
    r = 0.115 # m/s

    xGuass, wGuass = roots_legendre(2)

    thetaVecSize = 20
    thetaVec = np.linspace(0, 2*np.pi - (2*np.pi/thetaVecSize), num=thetaVecSize)

    sigma = 5000
    ztMinMain = 0
    ztMaxMain = 15000

    # =============================================================================
    # PF params
    # =============================================================================

    Theta_u = windDir+Theta_range
    Theta_l = windDir-Theta_range

    # =============================================================================
    # Create PF
    # =============================================================================

    print(X_u , X_l, mapPercent)
    print(Y_u , Y_l)
    print(Z_u , Z_l)
    print(Theta_u , Theta_l, percentTheta)
    print(Q_u , Q_l, percentQ)
    print(V_u , V_l, percentV)
    print(Dy_u , Dy_l, percentDy)
    print(Dz_u , Dz_l, percentDz)
    print(pdf_std, NumOfParticles)

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

    # =============================================================================
    # Saving data
    # =============================================================================

    ATrueLocations = np.matrix([xPlumeLoc,yPlumeLoc])
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
    DesiredWaypoint.pose.position.x = R1.PoseVec[0]
    DesiredWaypoint.pose.position.y = R1.PoseVec[1]
    DesiredWaypoint.pose.position.z = 0.2
    static_tf.transform.translation.x = R1.PoseVec[0]
    static_tf.transform.translation.y = R1.PoseVec[1]
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

    k = 0
    while not rospy.is_shutdown():
        # Wait until new sensor reading is recieved
        while ((not rospy.is_shutdown() and not sensorMsg_cb_flag) ):
            # print("Here")
            rate.sleep()
            global_waypoint_pub.publish(DesiredWaypoint);
            br.sendTransform(static_tf)
            if sensorMsg_cb_flag:
                break

        sensorMsg_cb_flag = False

        # Once a new reading has been reieved and your at the waypoint perform estimation and move robot
        k +=1

        # =============================================================================
        # Compute info waypoint
        # =============================================================================

        desiredInfoVelocities = []
        desiredInfoPositions  = []
        infoVec = []

        # R1.PoseVec = np.array([robotSensorReadingPoseGaden.local_x, robotSensorReadingPoseGaden.local_y, robotSensorReadingPoseGaden.local_z])

        for i in range(thetaVecSize):
            possibleVelocity = np.array([R1.VelVec[0] + r*np.cos(thetaVec[i]), R1.VelVec[1] + r*np.sin(thetaVec[i]), 0])
            possiblePosition = R1.updatedPoseSim(possibleVelocity, poseHeight = 0.2)
            desiredInfoVelocities.append(possibleVelocity)
            desiredInfoPositions.append(possiblePosition)

        # print()
        desiredInfoVelocities = np.array(desiredInfoVelocities)
        desiredInfoPositions = np.array(desiredInfoPositions)

        for i in range(len(desiredInfoPositions)):
            xInfoDes = desiredInfoPositions[i,:]

            MutualInformation = informationAtXNewNB(xInfoDes, R1_Pf.xp, R1_Pf.wp, sigma, ztMinMain, ztMaxMain, xGuass, wGuass, Ahat)
            infoVec.append(MutualInformation)

        infoVec = np.array(infoVec)

        newVelocity = desiredInfoVelocities[np.argmax(infoVec),:]
        desiredPose = desiredInfoPositions[np.argmax(infoVec),:]

        R1.updatePose(1, newVelocity, poseHeight = 0.2)

        if R1.PoseVec[0] > xmax:
            R1.PoseVec[0] = xmax
        elif R1.PoseVec[0] < xmin:
            R1.PoseVec[0] = xmin

        if R1.PoseVec[1] > ymax:
            R1.PoseVec[1] = ymax
        elif R1.PoseVec[1] < ymin:
            R1.PoseVec[1] = ymin

        # 1. Get Robot Pose
        # Done globally

        # 2. Get sensor reading and modifiy it with found plumes
        z_t = ppm_reading - getReadingMultiPlume(x_t[0], x_t[1], x_t[2], Ahat)

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
            print("Found plume")
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
            R1_Pf.resetParticles()
            R1_Pf.wp = np.ones(NumOfParticles) * 1/NumOfParticles
            R1_Pf.updateParticlesFromPastMeasurementsNumbaNew(zVec, xVec, Ahat)

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

        A.append(np.array([Xfound, Yfound, Zfound, Thetafound, Qfound, Vfound, Dyfound, Dzfound]))
        alphaHat.append(gaussHatVec)
        stdVec.append(R1_Pf.stdL2Norm)

        # Move robot to next waypoint
        # basically
        # x_t = R1.PoseVec

        # Finish script when waypoint list runs out
        if k == 1600:
            break

        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = "map_gaden"
        static_tf.child_frame_id = "Robot_1/base_link"
        DesiredWaypoint.header.frame_id = "map_gaden"
        DesiredWaypoint.pose.position.x = R1.PoseVec[0]
        DesiredWaypoint.pose.position.y = R1.PoseVec[1]
        DesiredWaypoint.pose.position.z = 0.2
        static_tf.transform.translation.x = R1.PoseVec[0]
        static_tf.transform.translation.y = R1.PoseVec[1]
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


        marker = Marker()
        marker.header.frame_id = "map_gaden"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = "points";
        marker.id = 0
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = (0.025,0.025,0.025)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.75
        marker.pose.orientation.w = 0

        for i in range(NumOfParticles/10):
            p = Point()
            p.x = R1_Pf.xp[i,0]
            p.y = R1_Pf.xp[i,1]
            p.z = R1_Pf.xp[i,2]
            marker.points.append(p)



        particlesRVIZ.publish(marker)
        # Move robot to desired waypoint and wait for next reading
        global_waypoint_pub.publish(DesiredWaypoint)
        br.sendTransform(static_tf)

        rate.sleep()

    print("Simulation has finished")
    rospack = rospkg.RosPack()

    pickle_dictionary = {'k': kVec, 'xVec': xVec, 'A': A, 'alphaHat': alphaHat, 'z': zVec, 'ATrueLocations': ATrueLocations, 'stdVec': stdVec, 'simType': simType}

    simNumberPadded = str(simNumber).zfill(3)

    fullDirStringName = rospack.get_path('platform_real') + '/info/' + simNumberPadded
    print(fullDirStringName)

    if saveResults:
        pickle.dump( pickle_dictionary, open(fullDirStringName, "wb" ) )
        print("Data has been saved")

    os.system('pkill roslaunch')



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
