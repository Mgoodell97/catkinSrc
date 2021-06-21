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

quad_agent_msg_dict = {}
quad_agent_msg_dict_previous = {}
quad_agent_msg_flag = {}

##################
# Functions
##################



##################
# Callbacks
##################

def SELF_gaussSensor_cb(gaussMsg):
    global SELF_current_reading_gauss
    SELF_current_reading_gauss = gaussMsg

def SELF_gadenSensor_cb(gadenMsg):
    global SELF_current_reading_gaden
    SELF_current_reading_gaden = gadenMsg

def SELF_MPS_Sensor_cb(MPS_Msg):
    global SELF_current_reading_MPS
    SELF_current_reading_MPS = MPS_Msg

def SELF_location_gauss_get(loc_read_all): # function(variable to store message data)
    global SELF_location_gauss
    SELF_location_gauss = loc_read_all

def SELF_location_gaden_get(loc_read_gaden):
    global SELF_location_gaden
    SELF_location_gaden = loc_read_gaden

def state_cb(state_sub):
    global current_state
    current_state = state_sub

def agent_mle_gauss_cb(agent_mle_gauss_msg_tmp):

    quad_agent_msg_dict_previous = quad_agent_msg_dict.copy()

    quad_agent_msg_dict[agent_mle_gauss_msg_tmp.agent_number] = agent_mle_gauss_msg_tmp.estimatedgaussian

    # Update PF with new particle and renormalize the weights
    if agent_mle_gauss_msg_tmp.agent_number not in quad_agent_msg_dict_previous.keys():
        quad_agent_msg_flag[agent_mle_gauss_msg_tmp.agent_number] = True # Raise flag to add particle in next loop

    elif quad_agent_msg_dict[agent_mle_gauss_msg_tmp.agent_number] != quad_agent_msg_dict_previous[agent_mle_gauss_msg_tmp.agent_number]:
        quad_agent_msg_flag[agent_mle_gauss_msg_tmp.agent_number] = True # Raise flag to add particle in next loop


##################
# Main
##################

def main():
    print('Initializing Particle Filter')

    rospy.Subscriber("mavros/state", State, state_cb) # subscribe to state of quad

    rospy.init_node('Particle_Filter') # name your node
    ## Initialization of Particles and Sensor ##
    rate = rospy.Rate(1)

    particle_params = rospy.get_param("particleFilter") # get params defined in launch file

    # Saves dictionary entries from launch file as variables
    for key,val in particle_params.items():
        exec(key + '=val')

    ##### Uses pulled variables and sets inital parameters #####
    # use launch file parameters to define parameters for the particles
    _PARTICLE_PARAMETERS = [[X_l, X_u],[Y_l, Y_u],[Z_l, Z_u],[Theta_l,Theta_u],[Q_l,Q_u],[V_l,V_u],[Dy_l,Dy_u],[Dz_l, Dz_u]]
    # define parameters for noise in particle filter
    _NOISE_STD_PARAMS = np.array((X_n,Y_n,Z_n,Theta_n,Q_n,V_n,Dy_n,Dz_n))
    # set number of particles
    _NUM_OF_PARTICLES = NumOfParticles
    # set number of particled to deal with impoverishment
    _IMP_PARTICLES = ImpovParticles

    particle_filter = Particle_Gen(_PARTICLE_PARAMETERS,_NUM_OF_PARTICLES,_NUM_OF_ROBOTS) # initializes particles
    sensors = Sensor(_NUM_OF_ROBOTS) # initializes sensor recording device

    # code for publishing particles
    particlesMsg = particles() # message type for particle publishing
    MLE_Msg = AgentEstimatedGaussian()

    if simulation: #True
        if PlumeType == "gaussian": # if gaussian env subscribe to gaussian messages
            rospy.Subscriber("gaussianReading", gaussian, SELF_gaussSensor_cb)
            rospy.Subscriber("true_position", PoseStamped, SELF_location_gauss_get)

        if PlumeType == "gaden": # if gaden subscribe to gaden messages
            rospy.Subscriber("Sensor_reading", gas_sensor, SELF_gadenSensor_cb)
            rospy.Subscriber("Sensor_display", Marker, SELF_location_gaden_get)
    else : #False
        rospy.Subscriber("MPS_Sensor_reading", MPS, SELF_MPS_Sensor_cb)
        rospy.Subscriber("true_position", PoseStamped, SELF_location_gauss_get) # might need to change

    # if simulation: #True
    if quadID == 1:
        rospy.Subscriber("/agent_mle_gauss_data1", AgentEstimatedGaussian, agent_mle_gauss_cb)

        mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)

    if quadID == 2:
        rospy.Subscriber("/agent_mle_gauss_data2", AgentEstimatedGaussian, agent_mle_gauss_cb)

        mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)

    if quadID == 3:
        rospy.Subscriber("/agent_mle_gauss_data3", AgentEstimatedGaussian, agent_mle_gauss_cb)

        mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)
    # else : #False


    particlesPublisher = rospy.Publisher('particles', particles, queue_size=5)
    pub = rospy.Publisher("estimatedGaussian", estimatedGaussian, queue_size=10)
    # maxConcPublisher = rospy.Publisher('max_conc',max_conc, queue_size=5)

    while not rospy.is_shutdown(): # while roscore is running do this. if not, stop... Executes Function
        # global location_reading
        while not current_state.armed: # waits until the quad is armed before starting particle filtering
            rospy.sleep(1)

        if simulation: #True
            if PlumeType == "gaussian": # If env is gauss then grab gauss message data
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
        else: # simulation == False
            chem_reading_self = SELF_current_reading_MPS.pressure
            sensors.reading(SELF_current_reading_MPS.pressure,SELF_location_gauss.pose.position.x,SELF_location_gauss.pose.position.y,SELF_location_gauss.pose.position.z) # saves information gathered in sensor history

            chem_readings = np.array([[SELF_current_reading_MPS.pressure]])
            x_locations = np.array([[SELF_location_gauss.pose.position.x]])
            y_locations = np.array([[SELF_location_gauss.pose.position.y]])
            z_locations = np.array([[SELF_location_gauss.pose.position.z]])

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
            mle_gauss_pub1.publish(MLE_Msg)
            mle_gauss_pub2.publish(MLE_Msg)


            LLE_idx = np.argmin(particle_filter.prob_df)

            for key, value in quad_agent_msg_flag.items():
                if value:
                    LLE_idx = np.argmin(particle_filter.prob_df)

                    particle_filter.particles[0][LLE_idx] = quad_agent_msg_dict[key].X
                    particle_filter.particles[1][LLE_idx] = quad_agent_msg_dict[key].Y
                    particle_filter.particles[2][LLE_idx] = quad_agent_msg_dict[key].Z
                    particle_filter.particles[3][LLE_idx] = quad_agent_msg_dict[key].Theta
                    particle_filter.particles[4][LLE_idx] = quad_agent_msg_dict[key].Q
                    particle_filter.particles[5][LLE_idx] = quad_agent_msg_dict[key].V
                    particle_filter.particles[6][LLE_idx] = quad_agent_msg_dict[key].Dy
                    particle_filter.particles[7][LLE_idx] = quad_agent_msg_dict[key].Dz
                    particle_filter.prob_df[LLE_idx]      = quad_agent_msg_dict[key].W
                    particle_filter.normalize()

                    quad_agent_msg_flag[key] = False
                    # print "Added agent's", key, "particle"
                    # print (quad_agent_msg_dict[key])


            # Seperate noise and resampling step...
            particle_filter.state_transition(_NOISE_STD_PARAMS)

            resamp_check = 1/sum((particle_filter.prob_df**2))
            # print('Resamp Check: ' + str(resamp_check))
            # print('np/2: ' +str(_NUM_OF_PARTICLES/2))
            if resamp_check <= _NUM_OF_PARTICLES/2:
                particle_filter.resample(_NUM_OF_PARTICLES, _IMP_PARTICLES)

            rate.sleep() # have you met the rospy.Rate? if not wait some amount of time.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# _PARTICLE_PARAMETERS_GAUS = [[0,50],[0, 50],[2,2],[3*np.pi/2,3*np.pi/2],[50000, 50000],[1, 1],[.15,.15],[.15,.15]] #x,y,z,theta,Q,V,Dy,Dz
# _PARTICLE_PARAMETERS_GADEN = [[0,50],[0, 50],[2,2],[3*np.pi/2,3*np.pi/2],[43659, 43659],[.828, .828],[1.03697,1.03697],[.00007265,.00007265]] #x,y,z,theta,Q,V,Dy,Dz
# _PARTICLE_PARAMETERS_GADEN = [[0,50],[0, 50],[2,2],[3*np.pi/2,3*np.pi/2],[43659, 43659],[.828, .828],[1.03697,1.03697],[.00007265,.00007265]] #x,y,z,theta,Q,V,Dy,Dz

#_PARTICLE_PARAMETERS = [[0,50],[0, 50],[2,2],[3*np.pi/2,3*np.pi/2],[50000, 50000],[1, 1],[.15,.15],[.15,.15]] #x,y,z,theta,Q,V,Dy,Dz
# _PARTICLE_PARAMETERS_GADEN = [[0,50],[0,50],[2,2],[3*np.pi/2,3*np.pi/2],[.1, 25],[.01, 10],[.1,5],[.1,10]] #x,y,z,theta,Q,V,Dy,Dz
# _PARTICLE_PARAMETERS_GADEN_ratebased = [[0,50],[0,50],[2,2],[3*np.pi/2,3*np.pi/2],[.1, 5],[.01, 10],[.1,5],[.1,10],[1, 1]]


# _PARTICLE_PARAMETERS = [[0, 50],[0, 50],[0,4],[0,6*np.pi/4],[90000, 110000],[0, 8],[20,60],[.005, .025]]

#_PDF_STD_DIST = 75

# # norm noise #
# x_noise = 1
# y_noise = 1
# z_noise = 0
# theta_noise = 0
# Q_noise = .05
# V_noise = .02
# Dy_noise = .025
# Dz_noise = 0.025

# x_noise = .25
# y_noise = .25
# z_noise = 0
# theta_noise = 0
# Q_noise = 0
# V_noise = 0
# Dy_noise = 0
# Dz_noise = 0

# x_noise = 0
# y_noise = 0
# z_noise = 0
# theta_noise = 0
# Q_noise = 250
# V_noise = .05
# Dy_noise = .025
# Dz_noise = .025



# # ratebased noise #
# x_noise = 1
# y_noise = 1
# z_noise = 0
# theta_noise = 0
# Q_noise = .02
# V_noise = .02
# D_noise = .02
# tau_noise = .02
# a_noise = 0
#
# _NOISE_STD_PARAMS_ratebased = np.array((x_noise,y_noise,z_noise,theta_noise,Q_noise,V_noise,D_noise,tau_noise,a_noise))

# chem_reading = gaussian()


# controllerValues.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# controllerValues.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# def chem_sens_cb(raw_readings_msg): # function(variable to store message data)
#     global chem_reading
#     chem_reading = raw_readings_msg
