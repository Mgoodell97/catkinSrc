#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
#import numpy
from math import cos, sin, pi, acos, sqrt, exp
import matplotlib.pyplot as plt
import random
import numpy as np
from scipy.stats import norm


# from sensor_msgs.msg import Joy

from quadnodes.msg import gaussian
from olfaction_msgs.msg import gas_sensor
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from mps_driver.msg import MPS

from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles

from enif_iuc.msg import AgentEstimatedGaussian

from mavros_msgs.msg import State #include <mavros_msgs/State.h>

##################
# Global variables
##################

global chem_reading
global current_state

plumeMsg = estimatedGaussian()

robotSensorReadingPoseGauss_R1 = gaussian() # chem reading in gauss env
robotSensorReadingPoseGauss_R2 = gaussian() # chem reading in gauss env
robotSensorReadingPoseGauss_R3 = gaussian() # chem reading in gauss env
robotSensorReadingPoseGauss_R4 = gaussian() # chem reading in gauss env
robotSensorReadingPoseGauss_R5 = gaussian() # chem reading in gauss env

robotSensorReadingPoseMPS_R1 = MPS() # chem reading in gauss env
robotSensorReadingPoseMPS_R2 = MPS() # chem reading in gauss env
robotSensorReadingPoseMPS_R3 = MPS() # chem reading in gauss env
robotSensorReadingPoseMPS_R4 = MPS() # chem reading in gauss env
robotSensorReadingPoseMPS_R5 = MPS() # chem reading in gauss env

robotSensorReadingPoseGADEN_R1 = gas_sensor() # chem reading in gauss env
robotSensorReadingPoseGADEN_R2 = gas_sensor() # chem reading in gauss env
robotSensorReadingPoseGADEN_R3 = gas_sensor() # chem reading in gauss env
robotSensorReadingPoseGADEN_R4 = gas_sensor() # chem reading in gauss env
robotSensorReadingPoseGADEN_R5 = gas_sensor() # chem reading in gauss env

current_state = State() # message type for current state of the quad

global quadID
global printID
global quad_agent_msg_dict
global quad_agent_msg_dict_previous
quad_agent_msg_dict = {}
quad_agent_msg_dict_previous = {}

sensorMsg_cb_flag = False;

##################
# Functions
##################

def gaussFunc(xFunc, yFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    con = (QFunc/(4 * pi * xFunc * sqrt(DyFunc*DzFunc))) * exp( -vFunc/(4*xFunc) * ((yFunc**2)/DyFunc + (zFunc**2)/DzFunc))
    return con * 1000 # convert from kg/m^3 to ppm

def getReading(xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc):
    # Rotate frame

    Stheta = sin(thetaFunc)
    Ctheta = cos(thetaFunc)

    xRobotRotated = (Ctheta  * xRobotDef + Stheta * yRobotDef + -Ctheta * xPlumeFunc - Stheta * yPlumeFunc)
    yRobotRotated = (-Stheta * xRobotDef + Ctheta * yRobotDef +  Stheta * xPlumeFunc - Ctheta * yPlumeFunc)

    if xRobotRotated <= 0:
        reading = 0
    else:
        reading = gaussFunc(xRobotRotated,yRobotRotated,zFunc,QFunc,vFunc,DyFunc,DzFunc)

    return reading

def observerEquation(robotPose, xp):
    N = len(xp)
    yp = np.zeros(N)
    for i in range(N):
                             # 0    1   2     3    4    5   6    7
        # xp = np.concatenate((xp, yp, zp, thetap, Qp, vp, Dyp, Dzp), axis=1)
        # (xRobotDef, yRobotDef, thetaFunc, xPlumeFunc, yPlumeFunc, zFunc, QFunc, vFunc, DyFunc, DzFunc)
        yp[i] = getReading(robotPose[0], robotPose[1], xp[i,3], xp[i,0], xp[i,1], robotPose[2] - xp[i,2], xp[i,4], xp[i,5], xp[i,6], xp[i,7])
    return yp

def resamplingOld(xp, wp):
    N = len(wp)
    wc = np.cumsum(wp)

    # print("N :", N)
    # print("wc :", wc)
    # print("(1/N) :", (1.0/N))
    uj = (1.0/N) * np.random.rand(1)[0] # random number from 0 to np^-1

    # print("uj :", uj)

    i = 0
    ind = np.zeros(N,dtype = int)
    for j in range(N):
        while(uj > wc[i]):
            i+=1
        ind[j] = i
        uj = uj + 1.0/N

    # print("ind :", ind)

    xNew = xp[ind]

    wNew = np.ones(N)/N
    # return xNew, wNew
    return xNew, wNew

def pinWheelResampler(xp, wp):

    N = len(wp)
    wc = np.cumsum(wp)
    # print(N)
    uj = (1.0/N) * np.random.rand(1)[0] # random number from 0 to np^-1

    i = 0
    ind = np.zeros(N,dtype = int)

    for j in range(N):
        while(uj > wc[i]):
            i+=1
        ind[j] = i
        uj = uj + 1.0/N

    xNew = xp[ind]

    return xNew

def resampling(xp, wp, NumberOfParticlesToReset, randomStateBoundsVec, NumOfParticles):
    #               0          1          2              3             4         5          6           7
    # np.array([[X_l, X_u],[Y_l, Y_u],[Z_l, Z_u],[Theta_l,Theta_u],[Q_l,Q_u],[V_l,V_u],[Dy_l,Dy_u],[Dz_l, Dz_u]])

    wpIndex = np.argsort(wp)
    wpHigh = wp[wpIndex[NumberOfParticlesToReset:]]
    xpHigh = xp[wpIndex[NumberOfParticlesToReset:]]

    # Resample low weighted particles
    xpLow     = np.asmatrix(np.random.uniform(randomStateBoundsVec[0,0], randomStateBoundsVec[0,1], NumberOfParticlesToReset))
    ypLow     = np.asmatrix(np.random.uniform(randomStateBoundsVec[1,0], randomStateBoundsVec[1,1], NumberOfParticlesToReset))
    zpLow     = np.asmatrix(np.random.uniform(randomStateBoundsVec[2,0], randomStateBoundsVec[2,1], NumberOfParticlesToReset))
    thetapLow = np.asmatrix(np.random.uniform(randomStateBoundsVec[3,0], randomStateBoundsVec[3,1], NumberOfParticlesToReset))
    QpLow     = np.asmatrix(np.random.uniform(randomStateBoundsVec[4,0], randomStateBoundsVec[4,1], NumberOfParticlesToReset))
    vpLow     = np.asmatrix(np.random.uniform(randomStateBoundsVec[5,0], randomStateBoundsVec[5,1], NumberOfParticlesToReset))
    DypLow    = np.asmatrix(np.random.uniform(randomStateBoundsVec[6,0], randomStateBoundsVec[6,1], NumberOfParticlesToReset))
    DzpLow    = np.asmatrix(np.random.uniform(randomStateBoundsVec[7,0], randomStateBoundsVec[7,1], NumberOfParticlesToReset))

    xpNewLow = np.concatenate((xpLow.T, ypLow.T, zpLow.T, thetapLow.T, QpLow.T, vpLow.T, DypLow.T, DzpLow.T), axis=1)

    # Resample high weighted particles
    xpNewHigh = pinWheelResampler(xpHigh, wpHigh/sum(wpHigh));

    xpNew = np.concatenate((xpNewLow, xpNewHigh), axis=0)
    wpNew = np.ones(NumOfParticles)/NumOfParticles

    return xpNew, wpNew

##################
# Callbacks
##################

def gaussSensorAndPose_cb_R1(gaussMsg):
    global robotSensorReadingPoseGauss_R1
    global sensorMsg_cb_flag
    robotSensorReadingPoseGauss_R1 = gaussMsg
    sensorMsg_cb_flag = True

def gaussSensorAndPose_cb_R2(gaussMsg):
    global robotSensorReadingPoseGauss_R2
    global sensorMsg_cb_flag
    robotSensorReadingPoseGauss_R2 = gaussMsg
    sensorMsg_cb_flag = True

def gaussSensorAndPose_cb_R3(gaussMsg):
    global robotSensorReadingPoseGauss_R3
    global sensorMsg_cb_flag
    robotSensorReadingPoseGauss_R3 = gaussMsg
    sensorMsg_cb_flag = True

def gaussSensorAndPose_cb_R4(gaussMsg):
    global robotSensorReadingPoseGauss_R4
    global sensorMsg_cb_flag
    robotSensorReadingPoseGauss_R4 = gaussMsg
    sensorMsg_cb_flag = True

def gaussSensorAndPose_cb_R5(gaussMsg):
    global robotSensorReadingPoseGauss_R5
    global sensorMsg_cb_flag
    robotSensorReadingPoseGauss_R5 = gaussMsg
    sensorMsg_cb_flag = True



def MPSSensorAndPose_cb_R1(MPSMsg):
    global robotSensorReadingPoseMPS_R1
    global sensorMsg_cb_flag
    robotSensorReadingPoseMPS_R1 = MPSMsg
    sensorMsg_cb_flag = True

def MPSSensorAndPose_cb_R2(MPSMsg):
    global robotSensorReadingPoseMPS_R2
    global sensorMsg_cb_flag
    robotSensorReadingPoseMPS_R2 = MPSMsg
    sensorMsg_cb_flag = True

def MPSSensorAndPose_cb_R3(MPSMsg):
    global robotSensorReadingPoseMPS_R3
    global sensorMsg_cb_flag
    robotSensorReadingPoseMPS_R3 = MPSMsg
    sensorMsg_cb_flag = True

def MPSSensorAndPose_cb_R4(MPSMsg):
    global robotSensorReadingPoseMPS_R4
    global sensorMsg_cb_flag
    robotSensorReadingPoseMPS_R4 = MPSMsg
    sensorMsg_cb_flag = True

def MPSSensorAndPose_cb_R5(MPSMsg):
    global robotSensorReadingPoseMPS_R5
    global sensorMsg_cb_flag
    robotSensorReadingPoseMPS_R5 = MPSMsg
    sensorMsg_cb_flag = True


def GADENSensorAndPose_cb_R1(GADENMsg):
    global robotSensorReadingPoseGADEN_R1
    global sensorMsg_cb_flag
    robotSensorReadingPoseGADEN_R1 = GADENMsg
    sensorMsg_cb_flag = True

def GADENSensorAndPose_cb_R2(GADENMsg):
    global robotSensorReadingPoseGADEN_R2
    global sensorMsg_cb_flag
    robotSensorReadingPoseGADEN_R2 = GADENMsg
    sensorMsg_cb_flag = True

def GADENSensorAndPose_cb_R3(GADENMsg):
    global robotSensorReadingPoseGADEN_R3
    global sensorMsg_cb_flag
    robotSensorReadingPoseGADEN_R3 = GADENMsg
    sensorMsg_cb_flag = True

def GADENSensorAndPose_cb_R4(GADENMsg):
    global robotSensorReadingPoseGADEN_R4
    global sensorMsg_cb_flag
    robotSensorReadingPoseGADEN_R4 = GADENMsg
    sensorMsg_cb_flag = True

def GADENSensorAndPose_cb_R5(GADENMsg):
    global robotSensorReadingPoseGADEN_R5
    global sensorMsg_cb_flag
    robotSensorReadingPoseGADEN_R5 = GADENMsg
    sensorMsg_cb_flag = True


# def agent_mle_gauss_cb(agent_mle_gauss_msg_tmp, particle_filter):
#     quad_agent_msg_dict_previous = quad_agent_msg_dict.copy()
#     quad_agent_msg_dict[agent_mle_gauss_msg_tmp.agent_number] = agent_mle_gauss_msg_tmp.estimatedgaussian
#
#     LLE_idx = np.argmin(particle_filter.prob_df)
#
#     global printID
#
#     # print(particle_filter.prob_df[LLE_idx], agent_mle_gauss_msg_tmp.estimatedgaussian.W)
#     # Update PF with new particle and renormalize the weights
#     if agent_mle_gauss_msg_tmp.agent_number not in quad_agent_msg_dict_previous.keys():
#
#         particle_filter.particles[0][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.X
#         particle_filter.particles[1][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Y
#         particle_filter.particles[2][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Z
#         particle_filter.particles[3][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Theta
#         particle_filter.particles[4][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Q
#         particle_filter.particles[5][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.V
#         particle_filter.particles[6][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Dy
#         particle_filter.particles[7][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Dz
#         particle_filter.prob_df[LLE_idx]      = agent_mle_gauss_msg_tmp.estimatedgaussian.W
#         particle_filter.normalize()
#         print "Agent", printID, "added agent's", agent_mle_gauss_msg_tmp.agent_number, "particle"
#
#     elif quad_agent_msg_dict[agent_mle_gauss_msg_tmp.agent_number] != quad_agent_msg_dict_previous[agent_mle_gauss_msg_tmp.agent_number]:
#         particle_filter.particles[0][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.X
#         particle_filter.particles[1][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Y
#         particle_filter.particles[2][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Z
#         particle_filter.particles[3][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Theta
#         particle_filter.particles[4][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Q
#         particle_filter.particles[5][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.V
#         particle_filter.particles[6][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Dy
#         particle_filter.particles[7][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Dz
#         particle_filter.prob_df[LLE_idx]      = agent_mle_gauss_msg_tmp.estimatedgaussian.W
#         particle_filter.normalize()
#         print "Agent", printID, "added agent's", agent_mle_gauss_msg_tmp.agent_number, "particle"

##################
# Main
##################

def main():
    print('Initializing Particle Filter')

    rospy.init_node('Particle_Filter') # name your node
    ## Initialization of Particles and Sensor ##
    rate = rospy.Rate(5)

    particle_params = rospy.get_param("particleFilter") # get params defined in launch file

    # Saves dictionary entries from launch file as variables
    for key,val in particle_params.items():
        exec(key + '=val')

    global printID
    printID = quadID

    plumeMsg = estimatedGaussian()
    particlesMsg = particles() # message type for particle publishing
    MLE_Msg = AgentEstimatedGaussian()

    print("X, Y, Z, Theta, Q, V, Dy, Dz")
    print([X_l, X_u],[Y_l, Y_u],[Z_l, Z_u],[Theta_l,Theta_u],[Q_l,Q_u],[V_l,V_u],[Dy_l,Dy_u],[Dz_l, Dz_u])
    print("Xn, Yn, Zn, Thetan, Qn, Vn, Dyn, Dzn")
    print(np.array((X_n,Y_n,Z_n,Theta_n,Q_n,V_n,Dy_n,Dz_n)))
    print(NumOfParticles)

    randomStateBoundsVec = np.array([[X_l, X_u],[Y_l, Y_u],[Z_l, Z_u],[Theta_l,Theta_u],[Q_l,Q_u],[V_l,V_u],[Dy_l,Dy_u],[Dz_l, Dz_u]])
    noiseVec = np.array([X_n,Y_n,Z_n,Theta_n,Q_n,V_n,Dy_n,Dz_n])

    if plumeType == 1:
        rospy.Subscriber("/Robot_1/gaussianReading", gaussian, gaussSensorAndPose_cb_R1)
        rospy.Subscriber("/Robot_2/gaussianReading", gaussian, gaussSensorAndPose_cb_R2)
        rospy.Subscriber("/Robot_3/gaussianReading", gaussian, gaussSensorAndPose_cb_R3)
        rospy.Subscriber("/Robot_4/gaussianReading", gaussian, gaussSensorAndPose_cb_R4)
        rospy.Subscriber("/Robot_5/gaussianReading", gaussian, gaussSensorAndPose_cb_R5)
    elif plumeType == 2:
        rospy.Subscriber("/Robot_1/Sensor_reading", gas_sensor, GADENSensorAndPose_cb_R1)
        rospy.Subscriber("/Robot_2/Sensor_reading", gas_sensor, GADENSensorAndPose_cb_R2)
        rospy.Subscriber("/Robot_3/Sensor_reading", gas_sensor, GADENSensorAndPose_cb_R3)
        rospy.Subscriber("/Robot_4/Sensor_reading", gas_sensor, GADENSensorAndPose_cb_R4)
        rospy.Subscriber("/Robot_5/Sensor_reading", gas_sensor, GADENSensorAndPose_cb_R5)
    elif plumeType == 3:
        rospy.Subscriber("/Robot_1/mps_data", MPS, MPSSensorAndPose_cb_R1)
        rospy.Subscriber("/Robot_2/mps_data", MPS, MPSSensorAndPose_cb_R2)
        rospy.Subscriber("/Robot_3/mps_data", MPS, MPSSensorAndPose_cb_R3)
        rospy.Subscriber("/Robot_4/mps_data", MPS, MPSSensorAndPose_cb_R4)
        rospy.Subscriber("/Robot_5/mps_data", MPS, MPSSensorAndPose_cb_R5)

    particlesPublisher = rospy.Publisher('particles', particles, queue_size=1)
    estimatedGaussianPublisher = rospy.Publisher("estimatedGaussian", estimatedGaussian, queue_size=10)

    robotPoseOld = np.array([-1000, -1000, -1000])
    resampleDistance = np.linalg.norm((np.array((X_u, Y_u, Z_u))*mapPercent) - (np.array((X_l, Y_l, Z_l))*mapPercent))

    # if quadID == 1:
    #     rospy.Subscriber("/agent_mle_gauss_data1", AgentEstimatedGaussian, agent_mle_gauss_cb, particle_filter)
    #
    #     mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub3 = rospy.Publisher('/agent_mle_gauss_data4', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub4 = rospy.Publisher('/agent_mle_gauss_data5', AgentEstimatedGaussian, queue_size=1)
    #
    # if quadID == 2:
    #     rospy.Subscriber("/agent_mle_gauss_data2", AgentEstimatedGaussian, agent_mle_gauss_cb, particle_filter)
    #
    #     mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub3 = rospy.Publisher('/agent_mle_gauss_data4', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub4 = rospy.Publisher('/agent_mle_gauss_data5', AgentEstimatedGaussian, queue_size=1)
    #
    # if quadID == 3:
    #     rospy.Subscriber("/agent_mle_gauss_data3", AgentEstimatedGaussian, agent_mle_gauss_cb, particle_filter)
    #
    #     mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub3 = rospy.Publisher('/agent_mle_gauss_data4', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub4 = rospy.Publisher('/agent_mle_gauss_data5', AgentEstimatedGaussian, queue_size=1)
    #
    # if quadID == 4:
    #     rospy.Subscriber("/agent_mle_gauss_data4", AgentEstimatedGaussian, agent_mle_gauss_cb, particle_filter)
    #
    #     mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub3 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub4 = rospy.Publisher('/agent_mle_gauss_data5', AgentEstimatedGaussian, queue_size=1)
    #
    # if quadID == 5:
    #     rospy.Subscriber("/agent_mle_gauss_data5", AgentEstimatedGaussian, agent_mle_gauss_cb, particle_filter)
    #
    #     mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub3 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)
    #     mle_gauss_pub4 = rospy.Publisher('/agent_mle_gauss_data4', AgentEstimatedGaussian, queue_size=1)

    #Generate particles
    xp     = np.asmatrix(np.random.uniform(randomStateBoundsVec[0,0], randomStateBoundsVec[0,1], NumOfParticles))
    yp     = np.asmatrix(np.random.uniform(randomStateBoundsVec[1,0], randomStateBoundsVec[1,1], NumOfParticles))
    zp     = np.asmatrix(np.random.uniform(randomStateBoundsVec[2,0], randomStateBoundsVec[2,1], NumOfParticles))
    thetap = np.asmatrix(np.random.uniform(randomStateBoundsVec[3,0], randomStateBoundsVec[3,1], NumOfParticles))
    Qp     = np.asmatrix(np.random.uniform(randomStateBoundsVec[4,0], randomStateBoundsVec[4,1], NumOfParticles))
    vp     = np.asmatrix(np.random.uniform(randomStateBoundsVec[5,0], randomStateBoundsVec[5,1], NumOfParticles))
    Dyp    = np.asmatrix(np.random.uniform(randomStateBoundsVec[6,0], randomStateBoundsVec[6,1], NumOfParticles))
    Dzp    = np.asmatrix(np.random.uniform(randomStateBoundsVec[7,0], randomStateBoundsVec[7,1], NumOfParticles))

    xp = np.concatenate((xp.T, yp.T, zp.T, thetap.T, Qp.T, vp.T, Dyp.T, Dzp.T), axis=1)
    wp = np.ones(NumOfParticles) * 1/NumOfParticles

    xStd     = np.std(xp[:,0])
    yStd     = np.std(xp[:,1])
    thetaStd = np.std(xp[:,3])
    stdVec = np.array([xStd, yStd, thetaStd])
    stdL2NormMax = np.linalg.norm(stdVec)

    # Form msgs
    plumeMsg.X           = sum(wp*xp[:,0])
    plumeMsg.Y           = sum(wp*xp[:,1])
    plumeMsg.Z           = sum(wp*xp[:,2])
    plumeMsg.Theta       = sum(wp*xp[:,3])
    plumeMsg.Q           = sum(wp*xp[:,4])
    plumeMsg.V           = sum(wp*xp[:,5])
    plumeMsg.Dy          = sum(wp*xp[:,6])
    plumeMsg.Dz          = sum(wp*xp[:,7])

    particlesMsg.X       = xp[:,0]
    particlesMsg.Y       = xp[:,1]
    particlesMsg.Z       = xp[:,2]
    particlesMsg.theta   = xp[:,3]
    particlesMsg.Q       = xp[:,4]
    particlesMsg.v       = xp[:,5]
    particlesMsg.Dy      = xp[:,6]
    particlesMsg.Dz      = xp[:,7]
    particlesMsg.weights = wp

    robotSensor = 1

    rospy.sleep(2)

    particlesPublisher.publish(particlesMsg)
    estimatedGaussianPublisher.publish(plumeMsg)

    rospy.sleep(2)

    while not rospy.is_shutdown(): # while roscore is running do this. if not, stop... Executes Function
        # print("============================================")
        # print(" ")

        while not rospy.get_param("/startTest") or not sensorMsg_cb_flag: # waits until the quad is armed before starting particle filtering
            rospy.sleep(1)
            # print(rospy.get_param("/startTest"), "   ,   ", sensorMsg_cb_flag)
            if rospy.get_param("/startTest") and sensorMsg_cb_flag:
                break

        # Get chemical reading
        if plumeType == 1:
            if robotSensor == 1:
                chem_reading = robotSensorReadingPoseGauss_R1.ppm
                robotPoseNew = np.array([robotSensorReadingPoseGauss_R1.x, robotSensorReadingPoseGauss_R1.y, robotSensorReadingPoseGauss_R1.z])
            elif robotSensor == 2:
                chem_reading = robotSensorReadingPoseGauss_R2.ppm
                robotPoseNew = np.array([robotSensorReadingPoseGauss_R2.x, robotSensorReadingPoseGauss_R2.y, robotSensorReadingPoseGauss_R2.z])
            elif robotSensor == 3:
                chem_reading = robotSensorReadingPoseGauss_R3.ppm
                robotPoseNew = np.array([robotSensorReadingPoseGauss_R3.x, robotSensorReadingPoseGauss_R3.y, robotSensorReadingPoseGauss_R3.z])
            elif robotSensor == 4:
                chem_reading = robotSensorReadingPoseGauss_R4.ppm
                robotPoseNew = np.array([robotSensorReadingPoseGauss_R4.x, robotSensorReadingPoseGauss_R4.y, robotSensorReadingPoseGauss_R4.z])
            elif robotSensor == 5:
                chem_reading = robotSensorReadingPoseGauss_R5.ppm
                robotPoseNew = np.array([robotSensorReadingPoseGauss_R5.x, robotSensorReadingPoseGauss_R5.y, robotSensorReadingPoseGauss_R5.z])

        elif plumeType == 2:
            if robotSensor == 1:
                chem_reading = robotSensorReadingPoseGADEN_R1.raw
                robotPoseNew = np.array([robotSensorReadingPoseGADEN_R1.local_x, robotSensorReadingPoseGADEN_R1.local_y, robotSensorReadingPoseGADEN_R1.local_z])
            elif robotSensor == 2:
                chem_reading = robotSensorReadingPoseGADEN_R2.raw
                robotPoseNew = np.array([robotSensorReadingPoseGADEN_R2.local_x, robotSensorReadingPoseGADEN_R2.local_y, robotSensorReadingPoseGADEN_R2.local_z])
            elif robotSensor == 3:
                chem_reading = robotSensorReadingPoseGADEN_R3.raw
                robotPoseNew = np.array([robotSensorReadingPoseGADEN_R3.local_x, robotSensorReadingPoseGADEN_R3.local_y, robotSensorReadingPoseGADEN_R3.local_z])
            elif robotSensor == 4:
                chem_reading = robotSensorReadingPoseGADEN_R4.raw
                robotPoseNew = np.array([robotSensorReadingPoseGADEN_R4.local_x, robotSensorReadingPoseGADEN_R4.local_y, robotSensorReadingPoseGADEN_R4.local_z])
            elif robotSensor == 5:
                chem_reading = robotSensorReadingPoseGADEN_R5.raw
                robotPoseNew = np.array([robotSensorReadingPoseGADEN_R5.local_x, robotSensorReadingPoseGADEN_R5.local_y, robotSensorReadingPoseGADEN_R5.local_z])

        elif plumeType == 3:
            if robotSensor == 1:
                chem_reading = robotSensorReadingPoseMPS_R1.pressure
                robotPoseNew = np.array([robotSensorReadingPoseMPS_R1.local_x, robotSensorReadingPoseMPS_R1.local_y, robotSensorReadingPoseMPS_R1.local_z])
            elif robotSensor == 2:
                chem_reading = robotSensorReadingPoseMPS_R2.pressure
                robotPoseNew = np.array([robotSensorReadingPoseMPS_R2.local_x, robotSensorReadingPoseMPS_R2.local_y, robotSensorReadingPoseMPS_R2.local_z])
            elif robotSensor == 3:
                chem_reading = robotSensorReadingPoseMPS_R3.pressure
                robotPoseNew = np.array([robotSensorReadingPoseMPS_R3.local_x, robotSensorReadingPoseMPS_R3.local_y, robotSensorReadingPoseMPS_R3.local_z])
            elif robotSensor == 4:
                chem_reading = robotSensorReadingPoseMPS_R4.pressure
                robotPoseNew = np.array([robotSensorReadingPoseMPS_R4.local_x, robotSensorReadingPoseMPS_R3.local_y, robotSensorReadingPoseMPS_R4.local_z])
            elif robotSensor == 5:
                chem_reading = robotSensorReadingPoseMPS_R5.pressure
                robotPoseNew = np.array([robotSensorReadingPoseMPS_R5.local_x, robotSensorReadingPoseMPS_R5.local_y, robotSensorReadingPoseMPS_R5.local_z])


        print("Robot:", robotSensor," data: ", chem_reading)

        # 1. Measurement prediction
        yi = observerEquation(robotPoseNew, xp)

        # 2. Likelihood
        xStd     = np.std(xp[:,0])
        yStd     = np.std(xp[:,1])
        thetaStd = np.std(xp[:,3])
        stdVec = np.array([xStd, yStd, thetaStd])
        stdL2Norm = np.linalg.norm(stdVec)
        if stdL2NormMax <= stdL2Norm:
            stdL2NormMax = stdL2Norm

        r = pdf_std * stdL2Norm / stdL2NormMax

        wp = wp * norm.pdf(chem_reading, yi, r)

        # 3. Normalization
        wpSum = sum(wp)
        if wpSum == 0:
            wp = np.ones(NumOfParticles) * 1/NumOfParticles
        else:
            wp = wp/wpSum

        # 4. Estimation and form messages
        plumeMsg.X           = sum(wp*xp[:,0])
        plumeMsg.Y           = sum(wp*xp[:,1])
        plumeMsg.Z           = sum(wp*xp[:,2])
        plumeMsg.Theta       = sum(wp*xp[:,3])
        plumeMsg.Q           = sum(wp*xp[:,4])
        plumeMsg.V           = sum(wp*xp[:,5])
        plumeMsg.Dy          = sum(wp*xp[:,6])
        plumeMsg.Dz          = sum(wp*xp[:,7])

        particlesMsg.X       = xp[:,0]
        particlesMsg.Y       = xp[:,1]
        particlesMsg.Z       = xp[:,2]
        particlesMsg.theta   = xp[:,3]
        particlesMsg.Q       = xp[:,4]
        particlesMsg.v       = xp[:,5]
        particlesMsg.Dy      = xp[:,6]
        particlesMsg.Dz      = xp[:,7]
        particlesMsg.weights = wp

        particlesPublisher.publish(particlesMsg)
        estimatedGaussianPublisher.publish(plumeMsg)

        # 5. Resampling
        resampleCheck = 1/sum(wp**2)

        dist = np.linalg.norm(robotPoseNew - robotPoseOld)

        # print(dist, resampleDistance)

        # if (resampleCheck <= NumOfParticles/2) and (dist >= resampleDistance):
        if (resampleCheck <= NumOfParticles/2):
            # print("resampling")
            xp, wp = resampling(xp, wp, NumberOfParticlesToReset, randomStateBoundsVec, NumOfParticles)

            # xp, wp = resamplingOld(xp,wp);

        # print("Resample")
        # xp, wp = resampling(xp, wp, NumberOfParticlesToReset, randomStateBoundsVec, NumOfParticles)

        # 6. Random process noise
        xNoise     = np.array([np.random.normal(0, X_n    , len(xp))]).T
        yNoise     = np.array([np.random.normal(0, Y_n    , len(xp))]).T
        zNoise     = np.array([np.random.normal(0, Z_n    , len(xp))]).T
        thetaNoise = np.array([np.random.normal(0, Theta_n, len(xp))]).T
        QNoise     = np.array([np.random.normal(0, Q_n    , len(xp))]).T
        VNoise     = np.array([np.random.normal(0, V_n    , len(xp))]).T
        DyNoise    = np.array([np.random.normal(0, Dy_n   , len(xp))]).T
        DzNoise    = np.array([np.random.normal(0, Dz_n   , len(xp))]).T

        noiseMatrix = np.concatenate((xNoise, yNoise, zNoise, thetaNoise, QNoise, VNoise, DyNoise, DzNoise), axis=1)

        xp = xp + noiseMatrix

        # print([X_l, X_u],[Y_l, Y_u],[Z_l, Z_u],[Theta_l,Theta_u],[Q_l,Q_u],[V_l,V_u],[Dy_l,Dy_u],[Dz_l, Dz_u])
        xp[:,0][xp[:,0]>X_u] = X_u
        xp[:,0][xp[:,0]<X_l] = X_l

        xp[:,1][xp[:,1]>Y_u] = Y_u
        xp[:,1][xp[:,1]<Y_l] = Y_l

        xp[:,2][xp[:,2]>Z_u] = Z_u
        xp[:,2][xp[:,2]<Z_l] = Z_l

        xp[:,3][xp[:,3]>Theta_u] = Theta_u
        xp[:,3][xp[:,3]<Theta_l] = Theta_l

        xp[:,4][xp[:,4]>Q_u] = Q_u
        xp[:,4][xp[:,4]<Q_l] = Q_l

        xp[:,5][xp[:,5]>V_u] = V_u
        xp[:,5][xp[:,5]<V_l] = V_l

        xp[:,6][xp[:,6]>Dy_u] = Dy_u
        xp[:,6][xp[:,6]<Dy_l] = Dy_l

        xp[:,7][xp[:,7]>Dz_u] = Dz_u
        xp[:,7][xp[:,7]<Dz_l] = Dz_l

        # vVec = np.concatenate((xpTemp, ypTemp, zpTemp, thetapTemp, QpTemp, VpTemp, DypTemp, DzpTemp), axis=1)

        # print(" ")

        robotPoseOld = robotPoseNew
        robotSensor+=1
        if robotSensor >=6:
            robotSensor = 1
        rate.sleep() # have you met the rospy.Rate? if not wait some amount of time.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
