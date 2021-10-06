#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Aug  6 16:54:18 2021

@author: nathan
"""

#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
#import numpy
from Particle_Filter_Utils import *
from wifi_models import ITU_indoor
# from sensor_msgs.msg import Joy

from quadnodes.msg import gaussian
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles
from particle_filter.msg import max_conc

from enif_iuc.msg import AgentEstimatedGaussian

from mavros_msgs.msg import State #include <mavros_msgs/State.h>

##################
# Global variables
##################

global current_state


Transmitter_location = [20,42]
Transmitter_dBm = 20.0 # range from 20-30 dBm
Transmitter_Mhz = 2462.0 # range from 2412-2472 and 5180-5700
floor_term = 8.0 # range (for 0,1,2,3-- residential: 0,4,8,12; office: 11,15,19,23)
N = 28.0 # range 24-30


const_term = -27.55


#_PARTICLE_PARAMETERS = [[-60,60],[-60,60],[20,30],[2412,2472],[0,23],[24,30]]
#_NOISE_STD_PARAMS = np.array([.5,.5,.1,3,.5,.1])
#_NUM_OF_PARTICLES = 50000
#_IMP_PARTICLES = 0
_NUM_OF_ROBOTS = 1


current_state = PoseStamped() # message type for current state of the quad

##################
# Functions
##################

def map( x,  in_min,  in_max,  out_min,  out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

##################
# Callbacks
##################

def state_cb(state_sub):
    global current_state
    current_state = state_sub

##################
# Main
##################

def main():
    print('Initializing Particle Filter')

    rospy.init_node('Particle_Filter') # name your node
    ## Initialization of Particles and Sensor ##
    rate = rospy.Rate(1)

    ##### Uses pulled variables and sets inital parameters #####
    # use launch file parameters to define parameters for the particles
#    _PARTICLE_PARAMETERS = [[X_l, X_u],[Y_l, Y_u],[Tx_l, Tx_u],[TMhz_l,TMhz_u],[FT_l,FT_u],[N_l,N_u]]
    _PARTICLE_PARAMETERS = [[0,120],[0,120],[20,30],[2412,2472],[0,23],[24,30]]

    fakeFlag = True

    # define parameters for noise in particle filter
#    _NOISE_STD_PARAMS = np.array((X_n,Y_n,Tx_n,TMhz_n,FT_n,N_n))
    _NOISE_STD_PARAMS = np.array([.5,.5,.1,3,.5,.1])

    # set number of particles
#    _NUM_OF_PARTICLES = NumOfParticles
    _NUM_OF_PARTICLES = 50000

    # set number of particled to deal with impoverishment
    _IMP_PARTICLES = 0

    particle_filter = Particle_Gen(_PARTICLE_PARAMETERS,_NUM_OF_PARTICLES,_NUM_OF_ROBOTS) # initializes particles
    sensors = Sensor(_NUM_OF_ROBOTS) # initializes sensor recording device

    # code for publishing particles

    rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, state_cb)

        # Need to add gaden plume sim

    particlesMsg = particles()

    particlesPublisher = rospy.Publisher('particles', particles, queue_size=1)
    # maxConcPublisher = rospy.Publisher('max_conc',max_conc, queue_size=5)

    X_ps = particle_filter.particles[0]
    Y_ps = particle_filter.particles[1]
    particlesMsg.X = X_ps
    particlesMsg.Y = Y_ps
    rospy.sleep(5)

    particlesPublisher.publish(particlesMsg)

    rospy.sleep(10)

    while not rospy.is_shutdown(): # while roscore is running do this. if not, stop... Executes Function
        x_locations = np.array([[map(current_state.pose.position.x*3.28084,0,14,0,120)]])
        y_locations = np.array([[map(current_state.pose.position.y*3.28084,0,14,0,120)]])
        # z_locations = np.array([[current_state.pose.position.z]])

        robot_location = np.array([x_locations,y_locations])

        RSSI_reading = ITU_indoor(robot_location,Transmitter_location,Transmitter_Mhz,Transmitter_dBm,floor_term,N,const_term)

        sensors.reading(RSSI_reading) # saves information gathered in sensor history
        particle_filter.likelihood_all(RSSI_reading, robot_location, const_term) # gets likelihod of each particle

        particle_probabilities = particle_filter.prob_df
        particle_probabilities = particle_probabilities.reshape((len(particle_probabilities),))
        X_ps = particle_filter.particles[0]
        Y_ps = particle_filter.particles[1]
        Tx_ps = particle_filter.particles[2]
        TMhz_ps = particle_filter.particles[3]
        FT_ps = particle_filter.particles[4]
        N_ps = particle_filter.particles[5]

        particlesMsg.X = X_ps
        particlesMsg.Y = Y_ps
#        particlesMsg.Z = Tx_ps
#        particlesMsg.theta = TMhz_ps
#        particlesMsg.Q = FT_ps
#        particlesMsg.v = N_ps

        particlesPublisher.publish(particlesMsg)
            # maxConcPublisher.publish(concMsg)
#        pub.publish(plumeMsg)

        resamp_check = 1/sum((particle_filter.prob_df**2))
            # print('Resamp Check: ' + str(resamp_check))
            # print('np/2: ' +str(_NUM_OF_PARTICLES/2))
        if resamp_check <= _NUM_OF_PARTICLES/2:
            particle_filter.resample(_NUM_OF_PARTICLES, _IMP_PARTICLES) # includes resampling check, does not guarentee a resampling

        particle_filter.state_transition(_NOISE_STD_PARAMS)


        rate.sleep() # have you met the rospy.Rate? if not wait some amount of time.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
