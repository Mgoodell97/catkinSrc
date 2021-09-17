#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
#import numpy
from Particle_Filter_Utils import *
import matplotlib.pyplot as plt
# from sensor_msgs.msg import Joy

from quadnodes.msg import gaussian
from olfaction_msgs.msg import gas_sensor
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from mps_driver.msg import MPS

from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles
from particle_filter.msg import max_conc

from enif_iuc.msg import AgentEstimatedGaussian

from mavros_msgs.msg import State #include <mavros_msgs/State.h>

##################
# Global variables
##################

global chem_reading
global current_state

_NUM_OF_ROBOTS = 1

plumeMsg = estimatedGaussian()

SELF_location_gauss = PoseStamped() # message type for location in gaussian env
SELF_current_reading_gauss = gaussian() # chem reading in gauss env

SELF_location_gaden = Marker() # message type for location in gaden env
SELF_current_reading_gaden = gas_sensor() # chem reading in gaden env

SELF_current_reading_MPS = MPS() # chem reading in gaden env

SELF_maxconc = max_conc()

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

from math import cos, sin, pi, acos, sqrt, exp

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

##################
# Callbacks
##################

def SELF_gaussSensor_cb(gaussMsg):
    global SELF_current_reading_gauss
    global sensorMsg_cb_flag
    SELF_current_reading_gauss = gaussMsg
    sensorMsg_cb_flag = True

def SELF_gadenSensor_cb(gadenMsg):
    global SELF_current_reading_gaden
    global sensorMsg_cb_flag
    SELF_current_reading_gaden = gadenMsg
    sensorMsg_cb_flag = True

def SELF_MPS_Sensor_cb(MPS_Msg):
    global SELF_current_reading_MPS
    global sensorMsg_cb_flag
    SELF_current_reading_MPS = MPS_Msg
    sensorMsg_cb_flag = True

def SELF_location_gauss_get(loc_read_all): # function(variable to store message data)
    global SELF_location_gauss
    SELF_location_gauss = loc_read_all

def SELF_location_gaden_get(loc_read_gaden):
    global SELF_location_gaden
    SELF_location_gaden = loc_read_gaden

def state_cb(state_sub):
    global current_state
    current_state = state_sub

def agent_mle_gauss_cb(agent_mle_gauss_msg_tmp, particle_filter):
    quad_agent_msg_dict_previous = quad_agent_msg_dict.copy()
    quad_agent_msg_dict[agent_mle_gauss_msg_tmp.agent_number] = agent_mle_gauss_msg_tmp.estimatedgaussian

    LLE_idx = np.argmin(particle_filter.prob_df)

    global printID

    # print(particle_filter.prob_df[LLE_idx], agent_mle_gauss_msg_tmp.estimatedgaussian.W)
    # Update PF with new particle and renormalize the weights
    if agent_mle_gauss_msg_tmp.agent_number not in quad_agent_msg_dict_previous.keys():

        particle_filter.particles[0][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.X
        particle_filter.particles[1][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Y
        particle_filter.particles[2][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Z
        particle_filter.particles[3][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Theta
        particle_filter.particles[4][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Q
        particle_filter.particles[5][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.V
        particle_filter.particles[6][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Dy
        particle_filter.particles[7][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Dz
        particle_filter.prob_df[LLE_idx]      = agent_mle_gauss_msg_tmp.estimatedgaussian.W
        particle_filter.normalize()
        print "Agent", printID, "added agent's", agent_mle_gauss_msg_tmp.agent_number, "particle"

    elif quad_agent_msg_dict[agent_mle_gauss_msg_tmp.agent_number] != quad_agent_msg_dict_previous[agent_mle_gauss_msg_tmp.agent_number]:
        particle_filter.particles[0][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.X
        particle_filter.particles[1][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Y
        particle_filter.particles[2][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Z
        particle_filter.particles[3][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Theta
        particle_filter.particles[4][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Q
        particle_filter.particles[5][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.V
        particle_filter.particles[6][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Dy
        particle_filter.particles[7][LLE_idx] = agent_mle_gauss_msg_tmp.estimatedgaussian.Dz
        particle_filter.prob_df[LLE_idx]      = agent_mle_gauss_msg_tmp.estimatedgaussian.W
        particle_filter.normalize()
        print "Agent", printID, "added agent's", agent_mle_gauss_msg_tmp.agent_number, "particle"

##################
# Main
##################

def main():
    print('Initializing Particle Filter')

    rospy.Subscriber("mavros/state", State, state_cb) # subscribe to state of quad

    rospy.init_node('Particle_Filter') # name your node
    ## Initialization of Particles and Sensor ##
    rate = rospy.Rate(5)

    particle_params = rospy.get_param("particleFilter") # get params defined in launch file

    # Saves dictionary entries from launch file as variables
    for key,val in particle_params.items():
        exec(key + '=val')

    global printID
    printID = quadID

    ##### Uses pulled variables and sets inital parameters #####
    # use launch file parameters to define parameters for the particles
    _PARTICLE_PARAMETERS = [[X_l, X_u],[Y_l, Y_u],[Z_l, Z_u],[Theta_l,Theta_u],[Q_l,Q_u],[V_l,V_u],[Dy_l,Dy_u],[Dz_l, Dz_u]]
    # define parameters for noise in particle filter
    _NOISE_STD_PARAMS = np.array((X_n,Y_n,Z_n,Theta_n,Q_n,V_n,Dy_n,Dz_n))
    # set number of particles
    _NUM_OF_PARTICLES = NumOfParticles
    # set number of particled to deal with impoverishment
    _IMP_PARTICLES = ImpovParticles

    particle_filter = Particle_Gen(_PARTICLE_PARAMETERS,_NUM_OF_PARTICLES,_NUM_OF_ROBOTS, pdf_std) # initializes particles
    sensors = Sensor(_NUM_OF_ROBOTS) # initializes sensor recording device

    # code for publishing particles
    particlesMsg = particles() # message type for particle publishing
    MLE_Msg = AgentEstimatedGaussian()

    if simulation: #True
        if PlumeType == "gaussian" and not Platform: # if gaussian env subscribe to gaussian messages
            rospy.Subscriber("gaussianReading", gaussian, SELF_gaussSensor_cb)
            rospy.Subscriber("true_position", PoseStamped, SELF_location_gauss_get)

        if PlumeType == "gaden": # if gaden subscribe to gaden messages
            rospy.Subscriber("Sensor_reading", gas_sensor, SELF_gadenSensor_cb)
            rospy.Subscriber("Sensor_display", Marker, SELF_location_gaden_get)

        if PlumeType == "gaussian" and Platform: # Platform gaussian
            rospy.Subscriber("gaussianReading", gaussian, SELF_gaussSensor_cb)
            fullStringName = "/mocap_node/Robot_" + str(int(quadID)) + "/pose"
            rospy.Subscriber(fullStringName, PoseStamped, SELF_location_gauss_get)

        # Need to add gaden plume sim

    else : #False
        rospy.Subscriber("MPS_Sensor_reading", MPS, SELF_MPS_Sensor_cb)
        rospy.Subscriber("true_position", PoseStamped, SELF_location_gauss_get) # might need to change

    # if simulation: #True
    if quadID == 1:
        rospy.Subscriber("/agent_mle_gauss_data1", AgentEstimatedGaussian, agent_mle_gauss_cb, particle_filter)

        mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub3 = rospy.Publisher('/agent_mle_gauss_data4', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub4 = rospy.Publisher('/agent_mle_gauss_data5', AgentEstimatedGaussian, queue_size=1)

    if quadID == 2:
        rospy.Subscriber("/agent_mle_gauss_data2", AgentEstimatedGaussian, agent_mle_gauss_cb, particle_filter)

        mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub3 = rospy.Publisher('/agent_mle_gauss_data4', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub4 = rospy.Publisher('/agent_mle_gauss_data5', AgentEstimatedGaussian, queue_size=1)

    if quadID == 3:
        rospy.Subscriber("/agent_mle_gauss_data3", AgentEstimatedGaussian, agent_mle_gauss_cb, particle_filter)

        mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub3 = rospy.Publisher('/agent_mle_gauss_data4', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub4 = rospy.Publisher('/agent_mle_gauss_data5', AgentEstimatedGaussian, queue_size=1)

    if quadID == 4:
        rospy.Subscriber("/agent_mle_gauss_data4", AgentEstimatedGaussian, agent_mle_gauss_cb, particle_filter)

        mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub3 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub4 = rospy.Publisher('/agent_mle_gauss_data5', AgentEstimatedGaussian, queue_size=1)

    if quadID == 5:
        rospy.Subscriber("/agent_mle_gauss_data5", AgentEstimatedGaussian, agent_mle_gauss_cb, particle_filter)

        mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub3 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub4 = rospy.Publisher('/agent_mle_gauss_data4', AgentEstimatedGaussian, queue_size=1)


    # else : #False


    particlesPublisher = rospy.Publisher('particles', particles, queue_size=1)
    pub = rospy.Publisher("estimatedGaussian", estimatedGaussian, queue_size=10)
    # maxConcPublisher = rospy.Publisher('max_conc',max_conc, queue_size=5)

    while not rospy.is_shutdown(): # while roscore is running do this. if not, stop... Executes Function

        if not Platform:
            while not current_state.armed: # waits until the quad is armed before starting particle filtering
                rospy.sleep(1)
        elif Platform:
            while not rospy.get_param("/startTest") or not sensorMsg_cb_flag: # waits until the quad is armed before starting particle filtering
                rospy.sleep(1)
                # print(rospy.get_param("/startTest"), "   ,   ", sensorMsg_cb_flag)
                if rospy.get_param("/startTest") and sensorMsg_cb_flag:
                    break

        if simulation: #True
            if PlumeType == "gaussian" and not Platform: # If env is gauss then grab gauss message data
                chem_reading_self = SELF_current_reading_gauss.ppm
                sensors.reading(SELF_current_reading_gauss.ppm,SELF_location_gauss.pose.position.x,SELF_location_gauss.pose.position.y,SELF_location_gauss.pose.position.z) # saves information gathered in sensor history

                chem_readings = np.array([[SELF_current_reading_gauss.ppm]])
                x_locations = np.array([[SELF_location_gauss.pose.position.x]])
                y_locations = np.array([[SELF_location_gauss.pose.position.y]])
                z_locations = np.array([[SELF_location_gauss.pose.position.z]])

            elif PlumeType == "gaden": # If env is gaden grab gaden message data
                chem_reading_self = SELF_current_reading_gaden.raw
                sensors.reading(SELF_current_reading_gaden.raw,SELF_location_gaden.pose.position.x,SELF_location_gaden.pose.position.y,SELF_location_gaden.pose.position.z) # saves information gathered in sensor history

                chem_readings = np.array([[SELF_current_reading_gaden.raw]])
                x_locations = np.array([[SELF_location_gaden.pose.position.x]])
                y_locations = np.array([[SELF_location_gaden.pose.position.y]])
                z_locations = np.array([[SELF_location_gaden.pose.position.z]])

            if PlumeType == "gaussian" and Platform: # If env is gauss then grab gauss message data
                chem_reading_self = SELF_current_reading_gauss.ppm
                sensors.reading(SELF_current_reading_gauss.ppm,SELF_current_reading_gauss.x,SELF_current_reading_gauss.y,SELF_current_reading_gauss.z) # saves information gathered in sensor history

                chem_readings = np.array([[SELF_current_reading_gauss.ppm]])
                x_locations = np.array([[SELF_current_reading_gauss.x]])
                y_locations = np.array([[SELF_current_reading_gauss.y]])
                z_locations = np.array([[SELF_current_reading_gauss.z]])

        else: # simulation == False
            chem_reading_self = SELF_current_reading_MPS.pressure
            sensors.reading(SELF_current_reading_MPS.pressure,SELF_location_gauss.pose.position.x,SELF_location_gauss.pose.position.y,SELF_location_gauss.pose.position.z) # saves information gathered in sensor history

            chem_readings = np.array([[SELF_current_reading_MPS.pressure]])
            x_locations = np.array([[SELF_location_gauss.pose.position.x]])
            y_locations = np.array([[SELF_location_gauss.pose.position.y]])
            z_locations = np.array([[SELF_location_gauss.pose.position.z]])

        # print(chem_reading_self)

        SELF_maxconc.X = sensors.max_conc_loc[0]
        SELF_maxconc.Y = sensors.max_conc_loc[1]
        SELF_maxconc.Z = sensors.max_conc_loc[2]
        SELF_maxconc.C = sensors.max_conc

        if chem_reading_self >= 0: # make parameter?
            particle_filter.likelihood(chem_readings,x_locations,y_locations,z_locations, SELF_maxconc)#, UAV1_maxconc, UAV2_maxconc, UAV3_maxconc) # gets likelihod of each particle

            # print(chem_reading_self)
            particle_probabilities = particle_filter.prob_df
            X_ps = particle_filter.particles[0]
            Y_ps = particle_filter.particles[1]
            Z_ps = particle_filter.particles[2]
            Theta_ps = particle_filter.particles[3]
            Q_ps = particle_filter.particles[4]
            V_ps = particle_filter.particles[5]
            Dy_ps = particle_filter.particles[6]
            Dz_ps = particle_filter.particles[7]

            X = sum(X_ps*particle_probabilities)
            Y = sum(Y_ps*particle_probabilities)
            Z = sum(Z_ps*particle_probabilities)
            Theta = sum(Theta_ps*particle_probabilities)
            Q = sum(Q_ps*particle_probabilities)
            V = sum(V_ps*particle_probabilities)
            Dy = sum(Dy_ps*particle_probabilities)
            Dz = sum(Dz_ps*particle_probabilities)

            MLE_idx = np.argmax(particle_probabilities)

            # Create msgs and publish information
            plumeMsg.X = X
            plumeMsg.Y = Y
            plumeMsg.Z = Z
            plumeMsg.Theta = Theta
            plumeMsg.Q = Q
            plumeMsg.V = V
            plumeMsg.Dy = Dy
            plumeMsg.Dz = Dz

            particlesMsg.X = X_ps
            particlesMsg.Y = Y_ps
            particlesMsg.Z = Z_ps
            particlesMsg.theta = Theta_ps
            particlesMsg.Q = Q_ps
            particlesMsg.v = V_ps
            particlesMsg.Dy = Dy_ps
            particlesMsg.Dz = Dz_ps
            particlesMsg.weights = particle_probabilities

            MLE_Msg.agent_number            = quadID
            MLE_Msg.estimatedgaussian.X     = X_ps[MLE_idx]
            MLE_Msg.estimatedgaussian.Y     = Y_ps[MLE_idx]
            MLE_Msg.estimatedgaussian.Z     = Z_ps[MLE_idx]
            MLE_Msg.estimatedgaussian.Theta = Theta_ps[MLE_idx]
            MLE_Msg.estimatedgaussian.Q     = Q_ps[MLE_idx]
            MLE_Msg.estimatedgaussian.V     = V_ps[MLE_idx]
            MLE_Msg.estimatedgaussian.Dy    = Dy_ps[MLE_idx]
            MLE_Msg.estimatedgaussian.Dz    = Dz_ps[MLE_idx]
            MLE_Msg.estimatedgaussian.W     = particle_probabilities[MLE_idx]

            particlesPublisher.publish(particlesMsg)
            # maxConcPublisher.publish(concMsg)
            pub.publish(plumeMsg)


            # print(chem_readings, "   ", getReading(x_locations, y_locations, 0, 2, 1.3716, 0.0, 0.000036, 0.447, 0.0075, 0.000075))

            # print(SELF_location_gauss)
            # print('Published estimate')

            mle_gauss_pub1.publish(MLE_Msg)
            mle_gauss_pub2.publish(MLE_Msg)

            # Seperate noise and resampling step...
            particle_filter.state_transition(_NOISE_STD_PARAMS)

            resamp_check = 1/sum((particle_filter.prob_df**2))
            # print('Resamp Check: ' + str(resamp_check))
            # print('np/2: ' +str(_NUM_OF_PARTICLES/2))
            # if resamp_check <= _NUM_OF_PARTICLES/2:
            particle_filter.resample(_NUM_OF_PARTICLES, _IMP_PARTICLES)

            rate.sleep() # have you met the rospy.Rate? if not wait some amount of time.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
