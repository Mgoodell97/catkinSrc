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

from particle_filter.msg import estimatedRatebased
from particle_filter.msg import particles
from particle_filter.msg import max_conc

from mavros_msgs.msg import State #include <mavros_msgs/State.h>

global chem_reading
global current_state

_NUM_OF_ROBOTS = 1

plumeMsg = estimatedRatebased()

SELF_location_gauss = PoseStamped() # message type for location in gaussian env
UAV1_location_gauss = PoseStamped() # message type for location in gaussian env
UAV2_location_gauss = PoseStamped() # message type for location in gaussian env
UAV3_location_gauss = PoseStamped() # message type for location in gaussian env

SELF_current_reading_gauss = gaussian() # chem reading in gauss env
UAV1_current_reading_gauss = gaussian() # chem reading in gauss env
UAV2_current_reading_gauss = gaussian() # chem reading in gauss env
UAV3_current_reading_gauss = gaussian() # chem reading in gauss env

SELF_location_gaden = Marker() # message type for location in gaden env
UAV1_location_gaden = Marker() # message type for location in gaden env
UAV2_location_gaden = Marker() # message type for location in gaden env
UAV3_location_gaden = Marker() # message type for location in gaden env

SELF_current_reading_gaden = gas_sensor() # chem reading in gaden env
UAV1_current_reading_gaden = gas_sensor() # chem reading in gaden env
UAV2_current_reading_gaden = gas_sensor() # chem reading in gaden env
UAV3_current_reading_gaden = gas_sensor() # chem reading in gaden env

UAV1_maxconc = max_conc()
UAV2_maxconc = max_conc()
UAV3_maxconc = max_conc()
current_state = State() # message type for current state of the quad

def SELF_gaussSensor_cb(gaussMsg):
    global SELF_current_reading_gauss
    current_reading_gauss = gaussMsg

def UAV1_gaussSensor_cb(gaussMsg):
    global UAV1_current_reading_gauss
    UAV1_current_reading_gauss = gaussMsg

def UAV2_gaussSensor_cb(gaussMsg):
    global UAV2_current_reading_gauss
    UAV2_current_reading_gauss = gaussMsg

def UAV3_gaussSensor_cb(gaussMsg):
    global UAV3_current_reading_gauss
    UAV3_current_reading_gauss = gaussMsg

def SELF_gadenSensor_cb(gadenMsg):
    global SELF_current_reading_gaden
    SELF_current_reading_gaden = gadenMsg

def UAV1_gadenSensor_cb(gadenMsg):
    global UAV1_current_reading_gaden
    UAV1_current_reading_gaden = gadenMsg

def UAV2_gadenSensor_cb(gadenMsg):
    global UAV2_current_reading_gaden
    UAV2_current_reading_gaden = gadenMsg

def UAV3_gadenSensor_cb(gadenMsg):
    global UAV3_current_reading_gaden
    UAV3_current_reading_gaden = gadenMsg

def SELF_location_gauss_get(loc_read_all): # function(variable to store message data)
    global SELF_location_gauss
    SELF_location_gauss = loc_read_all

def UAV1_location_gauss_get(loc_read_all): # function(variable to store message data)
    global UAV1_location_gauss
    UAV1_location_gauss = loc_read_all

def UAV2_location_gauss_get(loc_read_all): # function(variable to store message data)
    global UAV2_location_gauss
    UAV2_location_gauss = loc_read_all

def UAV3_location_gauss_get(loc_read_all): # function(variable to store message data)
    global UAV3_location_gauss
    UAV3_location_gauss = loc_read_all

def SELF_location_gaden_get(loc_read_gaden):
    global SELF_location_gaden
    SELF_location_gaden = loc_read_gaden

def UAV1_location_gaden_get(loc_read_gaden):
    global UAV1_location_gaden
    UAV1_location_gaden = loc_read_gaden

def UAV2_location_gaden_get(loc_read_gaden):
    global UAV2_location_gaden
    UAV2_location_gaden = loc_read_gaden

def UAV3_location_gaden_get(loc_read_gaden):
    global UAV3_location_gaden
    UAV3_location_gaden = loc_read_gaden

def state_cb(state_sub):
    global current_state
    current_state = state_sub

def UAV1_maxconc_cb(UAV1_maxconc_msg):
    global UAV1_maxconc
    UAV1_maxconc = UAV1_maxconc_msg

def UAV2_maxconc_cb(UAV2_maxconc_msg):
    global UAV2_maxconc
    UAV2_maxconc = UAV2_maxconc_msg

def UAV3_maxconc_cb(UAV3_maxconc_msg):
    global UAV3_maxconc
    UAV3_maxconc = UAV3_maxconc_msg

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

    if PlumeType == "gaussian": # if gaussian env subscribe to gaussian messages
        rospy.Subscriber("gaussianReading", gaussian, SELF_gaussSensor_cb)
        rospy.Subscriber("true_position", PoseStamped, SELF_location_gauss_get)

        rospy.Subscriber("/UAV1/gaussianReading", gaussian, UAV1_gaussSensor_cb)
        rospy.Subscriber("/UAV1/true_position", PoseStamped, UAV1_location_gauss_get)
        rospy.Subscriber("/UAV2/gaussianReading", gaussian, UAV2_gaussSensor_cb)
        rospy.Subscriber("/UAV2/true_position", PoseStamped, UAV2_location_gauss_get)
        rospy.Subscriber("/UAV3/gaussianReading", gaussian, UAV3_gaussSensor_cb)
        rospy.Subscriber("/UAV3/true_position", PoseStamped, UAV3_location_gauss_get)

    if PlumeType == "gaden": # if gaden subscribe to gaden messages
        rospy.Subscriber("Sensor_reading", gas_sensor, SELF_gadenSensor_cb)
        rospy.Subscriber("Sensor_display", Marker, SELF_location_gaden_get)

        rospy.Subscriber("/UAV1/Sensor_reading", gas_sensor, UAV1_gadenSensor_cb)
        rospy.Subscriber("/UAV1/Sensor_display", Marker, UAV1_location_gaden_get)
        rospy.Subscriber("/UAV2/Sensor_reading", gas_sensor, UAV2_gadenSensor_cb)
        rospy.Subscriber("/UAV2/Sensor_display", Marker, UAV2_location_gaden_get)
        rospy.Subscriber("/UAV3/Sensor_reading", gas_sensor, UAV3_gadenSensor_cb)
        rospy.Subscriber("/UAV3/Sensor_display", Marker, UAV3_location_gaden_get)

    rospy.Subscriber("/UAV1/max_conc", max_conc, UAV1_maxconc_cb)
    rospy.Subscriber("/UAV2/max_conc", max_conc, UAV2_maxconc_cb)
    rospy.Subscriber("/UAV3/max_conc", max_conc, UAV3_maxconc_cb)

    # code for publishing particles
    particlesMsg = particles() # message type for particle publishing
    concMsg = max_conc()

    particlesPublisher = rospy.Publisher('particles', particles, queue_size=5)
    maxConcPublisher = rospy.Publisher('max_conc',max_conc, queue_size=4)
    pub = rospy.Publisher("ratebasedEstimation", estimatedRatebased, queue_size=10)

    ##### Uses pulled variables and sets inital parameters #####
    # use launch file parameters to define parameters for the particles
    _PARTICLE_PARAMETERS = [[X_l, X_u],[Y_l, Y_u],[Z_l, Z_u],[Theta_l,Theta_u],[Q_l,Q_u],[V_l,V_u],[D_l,D_u],[Tau_l, Tau_u]]
    # define parameters for noise in particle filter
    _NOISE_STD_PARAMS = np.array((X_n,Y_n,Z_n,Theta_n,Q_n,V_n,D_n,Tau_n))
    # set number of particles
    _NUM_OF_PARTICLES = NumOfParticles
    # set number of particled to deal with impoverishment
    _IMP_PARTICLES = ImpovParticles

    particle_filter = Particle_Gen(_PARTICLE_PARAMETERS,_NUM_OF_PARTICLES,_NUM_OF_ROBOTS) # initializes particles
    sensors = Sensor(_NUM_OF_ROBOTS) # initializes sensor recording device


    while not rospy.is_shutdown(): # while roscore is running do this. if not, stop... Executes Function
        #rospy.loginfo("Throttle: %f     Roll: %f        Pitch: %f",controllerValues.axes[1],-controllerValues.axes[3],controllerValues.axes[4])
        # global location_reading
        while not current_state.armed: # waits until the quad is armed before starting particle filtering
            rospy.sleep(1)

        if PlumeType == "gaussian": # If env is gauss then grab gauss message data
            print('ERROR!!! CONSTANT GAUSSIAN PLUME IS NOT REALISTIC FOR RATEBASED MODEL!!!')
            chem_reading_self = SELF_current_reading_gauss.ppm
            x_location_self = SELF_location_gauss.pose.position.x
            y_location_self = SELF_location_gauss.pose.position.y
            z_location_self = SELF_location_gauss.pose.position.z
            chem_readings = np.array([[UAV1_current_reading_gauss.ppm, UAV2_current_reading_gauss.ppm, UAV3_current_reading_gauss.ppm]])
            x_locations = np.array([[UAV1_location_gauss.pose.position.x,UAV2_location_gauss.pose.position.x,UAV3_location_gauss.pose.position.x]])
            y_locations = np.array([[UAV1_location_gauss.pose.position.y,UAV2_location_gauss.pose.position.y,UAV3_location_gauss.pose.position.y]])
            z_locations = np.array([[UAV1_location_gauss.pose.position.z,UAV2_location_gauss.pose.position.z,UAV3_location_gauss.pose.position.z]])

        elif PlumeType == "gaden": # If env is gaden grab gaden message data
            chem_readings = np.array([[UAV1_current_reading_gaden.raw, UAV2_current_reading_gaden.raw, UAV3_current_reading_gaden.raw]])
            x_locations = np.array([[UAV1_location_gaden.pose.position.x,UAV2_location_gaden.pose.position.x,UAV3_location_gaden.pose.position.x]])
            y_locations = np.array([[UAV1_location_gaden.pose.position.y,UAV2_location_gaden.pose.position.y,UAV3_location_gaden.pose.position.y]])
            z_locations = np.array([[UAV1_location_gaden.pose.position.z,UAV2_location_gaden.pose.position.z,UAV3_location_gaden.pose.position.z]])
            chem_reading_self = SELF_current_reading_gaden.raw
            x_location_self = SELF_location_gaden.pose.position.x
            y_location_self = SELF_location_gaden.pose.position.y
            z_location_self = SELF_location_gaden.pose.position.z

        max_conc_quad = np.argmax(np.array([UAV1_maxconc.C,UAV2_maxconc.C,UAV3_maxconc.C]))

        if max_conc_quad == 0:
            max_conc_quad = UAV1_maxconc
        elif max_conc_quad == 1:
            max_conc_quad = UAV2_maxconc
        elif max_conc_quad == 2:
            max_conc_quad = UAV3_maxconc

        sensors.reading(chem_reading_self,x_location_self,y_location_self,z_location_self) # saves information gathered in sensor history

        particle_filter.likelihood_ratebased(chem_readings, x_locations , y_locations, z_locations, max_conc_quad) # gets likelihod of each particle
        particle_probabilities = particle_filter.prob_df
        X_ps = particle_filter.particles[0]
        Y_ps = particle_filter.particles[1]
        Z_ps = particle_filter.particles[2]
        Theta_ps = particle_filter.particles[3]
        Q_ps = particle_filter.particles[4]
        V_ps = particle_filter.particles[5]
        D_ps = particle_filter.particles[6]
        Tau_ps = particle_filter.particles[7]

        X = sum(X_ps*particle_probabilities)
        Y = sum(Y_ps*particle_probabilities)
        Z = sum(Z_ps*particle_probabilities)
        Theta = sum(Theta_ps*particle_probabilities)
        Q = sum(Q_ps*particle_probabilities)
        V = sum(V_ps*particle_probabilities)
        D = sum(D_ps*particle_probabilities)
        Tau = sum(Tau_ps*particle_probabilities)

        MLEparticle, MLE_idx  = particle_filter.MLE(1)
        LLEparticle, LLE_idx  = particle_filter.LLE(1)

        # print("Most Likely: " + str(MLEparticle))
        # print("Least Likely: " + str(LLEparticle))

        # Create msgs and publish information
        plumeMsg.X = X
        plumeMsg.Y = Y
        plumeMsg.Z = Z
        plumeMsg.Theta = Theta
        plumeMsg.Q = Q
        plumeMsg.V = V
        plumeMsg.D = D
        plumeMsg.Tau = Tau

        particlesMsg.X = particle_filter.particles[0]
        particlesMsg.Y = particle_filter.particles[1]
        particlesMsg.Z = particle_filter.particles[2]

        concMsg.X = sensors.max_conc_loc[0]
        concMsg.Y = sensors.max_conc_loc[1]
        concMsg.Z = sensors.max_conc_loc[2]
        concMsg.C = sensors.max_conc

        particlesPublisher.publish(particlesMsg)
        maxConcPublisher.publish(concMsg)
        pub.publish(plumeMsg)

        # Seperate noise and resampling step...
        particle_filter.state_transition(_NOISE_STD_PARAMS)

        resamp_check = 1/sum((particle_filter.prob_df**2))
        # print('Resamp Check: ' + str(resamp_check))
        # print('np/2: ' +str(_NUM_OF_PARTICLES/2))
        if resamp_check <= _NUM_OF_PARTICLES/2:
            particle_filter.resample(_NUM_OF_PARTICLES, _IMP_PARTICLES)

        # particle_filter.resamp_and_noise(_NOISE_STD_PARAMS,_NUM_OF_PARTICLES, _IMP_PARTICLES)



        # assume x and y is known later we will subscribe to topic matthew made

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
