# -*- coding: utf-8 -*-
"""
Created on Mon June 14 15:25:40 2021

Particle Filter Utilities to Localize Transmitter

@author: Nathan
"""
import time
import numpy as np
from numpy import inf
import math
from scipy.stats import norm
from scipy.stats import poisson

import matplotlib.pyplot as plt
import matplotlib.patches as patch

_X = 0
_Y = 1
_Z = 2

class Sensor():
    '''
    Sensor class- places inital sensor(s) in random location(s)
    '''
    def __init__(self, sens_number):# , vin_size=_VINYARD_SIZE, x_loc = None, y_loc = None, z_loc = None):
        self.history = np.array([])
        self.sens_number = sens_number
        self.max_RSSI = 0
#        self.max_conc_loc = [0,0,0]
    def reading(self, RSSI):
        '''
        Records Gas Concentration at sensor location
        '''
        self.history = np.append(self.history,RSSI) # keep track of past RSSI

        if RSSI>=self.max_RSSI:
            self.max_RSSIc = RSSI

class Particle_Gen():
    def __init__(self, estimated_parameters, num_of_particles, num_of_sensors):
        self.estimated_parameters = estimated_parameters
        self.num_of_particles = num_of_particles
        self.num_of_parameters = len(estimated_parameters)
        self.particles = np.zeros((self.num_of_parameters,self.num_of_particles))
        self.prior = np.ones((num_of_particles,1))
        self.num_of_sensors = num_of_sensors
        self.prob_df = np.ones((num_of_particles))/num_of_particles

        idx = 0
        for x in range(0,self.num_of_parameters):
            #print(x)
            self.particles[idx] = np.random.uniform(estimated_parameters[x][0],estimated_parameters[x][1],num_of_particles)
            idx += 1

    def particle_removal(self):
        particles_outside = np.ones((self.num_of_particles))

        for i in range(self.num_of_parameters):
            particles_outside = particles_outside*((self.particles[i] <= self.estimated_parameters[i][1])*(self.particles[i] >= self.estimated_parameters[i][0]))
        return particles_outside

    def pdf_thresh_function(self, RSSI_reading):
        #     if sensorval < .01:
        #         return .005
        #     elif sensorval < .1:
        #         return .05
        #     elif sensorval < 1:
        #         return .5
        #     elif sensorval < 10:
        #         return 5
        #     elif sensorval <50:
        #         return 25
        #     elif sensorval < 100:
        #         return 50
        #     elif sensorval <200:
        #         return 100
        #     else:
        #         return 500
        return 250

    def RSSI_convert(self, RSSI_reading):
        converted_RSSI = 10**(RSSI_reading/10)
        return converted_RSSI

    def find_particle_norms(self, x, y):
        norms = np.sqrt(x**2+y**2)
        return norms

    def likelihood_xy(self, RSSI_reading, Robot_location, Transmitter_dBm, Transmitter_Mhz, floor_term, N, const_term,i):
        '''
        Gets what sensor reading would be at the same location of the sensor
        within the plume of each particle
        '''
#        Mhz = 2462
#        floor_term = 8
#        N = 28
#        const_term = -27.55

        Xs = self.particles[0].reshape(self.num_of_particles,1) # x location of transmitter
        Ys = self.particles[1].reshape(self.num_of_particles,1) # y location of transmitter

        particle_distances = self.find_particle_norms(Robot_location[_X]-Xs,Robot_location[_Y]-Ys) # Finds distance between robot and each potential transmitter location


        Tran_mW = self.RSSI_convert(Transmitter_dBm)

        pdf_std = 1

        freq_term = 20*(np.log(Transmitter_Mhz)/np.log(10)) # megahertz
        dist_term = N*(np.log(particle_distances)/np.log(10))
        L = freq_term+dist_term+floor_term+const_term
        L = 10**(L/10)
        estimated_readings = Tran_mW/L
        estimated_readings = 10*np.log(estimated_readings)/np.log(10)
        print(RSSI_reading)
#        print(Tran_mWs)
#        print(particle_distances)


        print(estimated_readings)

        particle_likelihoods = norm.pdf(RSSI_reading,estimated_readings,pdf_std)


        if sum(particle_likelihoods)[0] == 0:
            print('NaN Error!')
        else:
            prob_df = particle_likelihoods*self.prior
            prob_df = prob_df/sum(prob_df)
            if sum(prob_df) > 0:
                self.prob_df = prob_df
        return


    def likelihood_xyTrans(self, RSSI_reading, Robot_location, Transmitter_Mhz, floor_term, N, const_term, i):
        '''
        Gets what sensor reading would be at the same location of the sensor
        within the plume of each particle
        '''
#        Mhz = 2462
#        floor_term = 8
#        N = 28
#        const_term = -27.55
#        particle_likelihoods = np.zeros((self.num_of_particles))


        Xs = self.particles[0].reshape(self.num_of_particles,1)
        Ys = self.particles[1].reshape(self.num_of_particles,1)
        particle_distances = self.find_particle_norms(Robot_location[_X]-Xs,Robot_location[_Y]-Ys) # Finds distance between robot and each potential transmitter location
#        particle_distances = self.particles[0].reshape(self.num_of_particles,1)
        Tran_dBms = self.particles[2].reshape(self.num_of_particles,1)
#        Tran_mWs = self.particles[2].reshape(self.num_of_particles,1)

        Tran_mWs = self.RSSI_convert(Tran_dBms)

#        plt.figure(i)
#        plt.hist(Tran_mWs)
#        plt.show()

        pdf_std = .2

        freq_term = 20*(np.log(Transmitter_Mhz)/np.log(10)) # megahertz
        dist_term = N*(np.log(particle_distances)/np.log(10))
        L = freq_term+dist_term+floor_term+const_term
        L = 10**(L/10)
        estimated_readings = Tran_mWs/L
        estimated_readings = 10*np.log(estimated_readings)/np.log(10)
#        print(RSSI_reading)
#        print(Tran_mWs)
#        print(particle_distances)


#        print(estimated_readings)

        particle_likelihoods = norm.pdf(RSSI_reading,estimated_readings,pdf_std)


        if sum(particle_likelihoods)[0] == 0:
            print('NaN Error!')
        else:
            prob_df = particle_likelihoods*self.prior
            prob_df = prob_df/sum(prob_df)
            if sum(prob_df) > 0:
                self.prob_df = prob_df
        return

    def likelihood_all(self, RSSI_reading, Robot_location, const_term):
        '''
        Gets what sensor reading would be at the same location of the sensor
        within the plume of each particle
        '''
#        Mhz = 2462
#        floor_term = 8
#        N = 28
#        const_term = -27.55
#        particle_likelihoods = np.zeros((self.num_of_particles))


        Xs = self.particles[0].reshape(self.num_of_particles,1)
        Ys = self.particles[1].reshape(self.num_of_particles,1)
        particle_distances = self.find_particle_norms(Robot_location[_X]-Xs,Robot_location[_Y]-Ys) # Finds distance between robot and each potential transmitter location
#        particle_distances = self.particles[0].reshape(self.num_of_particles,1)
        Tran_dBms = self.particles[2].reshape(self.num_of_particles,1)
        Transmitter_Mhz = self.particles[3].reshape(self.num_of_particles,1)
        floor_term = self.particles[4].reshape(self.num_of_particles,1)
        N = self.particles[5].reshape(self.num_of_particles,1)

#        Tran_mWs = self.particles[2].reshape(self.num_of_particles,1)

        Tran_mWs = self.RSSI_convert(Tran_dBms)

#        plt.figure(i)
#        plt.hist(Tran_mWs)
#        plt.show()

        pdf_std = .3

        freq_term = 20*(np.log(Transmitter_Mhz)/np.log(10)) # megahertz
        dist_term = N*(np.log(particle_distances)/np.log(10))
        L = freq_term+dist_term+floor_term+const_term
        L = 10**(L/10)
        estimated_readings = Tran_mWs/L
        estimated_readings = 10*np.log(estimated_readings)/np.log(10)
#        print(RSSI_reading)
#        print(Tran_mWs)
#        print(particle_distances)


#        print(estimated_readings)

        particle_likelihoods = norm.pdf(RSSI_reading,estimated_readings,pdf_std)


        if sum(particle_likelihoods)[0] == 0:
            print('NaN Error!')
        else:
            prob_df = particle_likelihoods*self.prior
            prob_df = prob_df/sum(prob_df)
            if sum(prob_df) > 0:
                self.prob_df = prob_df
        return


    def resample(self, num_of_particles, impov_particles):
        '''
        Generates new particles that are based on the likelihood of the current ones
        '''
        resamp_check = 1/sum((self.prob_df**2))

        if resamp_check >= self.num_of_particles/2:
            return

        print('resampling!')
        particle_indices = range(0,num_of_particles,1)

        resampled_index = np.random.choice(particle_indices, num_of_particles-impov_particles, replace=True, p = self.prob_df.reshape(num_of_particles,))
        new_imp_particles = np.zeros((self.num_of_parameters,impov_particles))
        new_particles = np.zeros((self.num_of_parameters,num_of_particles-impov_particles))

        for x in range(len(self.particles)):
            new_particles[x] = self.particles[x][resampled_index]
            new_imp_particles[x] = np.random.uniform(self.estimated_parameters[x][0],self.estimated_parameters[x][1],impov_particles)

        self.particles = np.concatenate((new_particles,new_imp_particles),axis=1)
        # keeps around old prior??? I think it starts uniform after resampling???
        #new_particles_prior = self.prob_df[resampled_index]
        #new_imp_particles_prior = (1/num_of_particles)*np.ones((impov_particles,))
        self.prior = np.ones((num_of_particles,1))

        return


    def state_transition(self, noise_std_params):
        for x in range(len(self.particles)):
            noise = np.random.normal(0, noise_std_params[x], self.num_of_particles)
            self.particles[x] = self.particles[x]+noise
        return

    def LLE(self, desired_number):
        idx = np.argpartition(self.prob_df, desired_number)
        least_likely_particles = self.particles[:,idx[:desired_number]]
        return least_likely_particles

    def MLE(self, desired_number):
        idx = np.argpartition(self.prob_df, -desired_number)
        most_likely_particles = self.particles[:,idx[-desired_number:]]
        return most_likely_particles

    def resamp_and_noise(self, noise_std_params, num_of_particles, impov_particles):
        '''
        Generates new particles that are based on the likelihood of the current ones
        '''
        particle_indices = range(0,num_of_particles,1)
        # print(num_of_particles-_IMP_PARTICLES)

        resampled_index = np.random.choice(particle_indices, num_of_particles-impov_particles, replace=True, p = self.prob_df)
        new_imp_particles = np.zeros((self.num_of_parameters,impov_particles))
        new_particles = np.zeros((self.num_of_parameters,num_of_particles-impov_particles))
        for x in range(len(self.particles)):
            noise = np.random.normal(0, noise_std_params[x], num_of_particles-impov_particles)
            new_particles[x] = self.particles[x][resampled_index]+noise
            new_imp_particles[x] = np.random.uniform(self.estimated_parameters[x][0],self.estimated_parameters[x][1],impov_particles)

        self.particles = np.concatenate((new_particles,new_imp_particles),axis=1)


        # new_particles_prior = self.prob_df[resampled_index]
        # new_imp_particles_prior = (1/num_of_particles)*np.ones((impov_particles,))
        self.prior = np.ones((num_of_particles,1))

        # generate noise and add it to new particles
        return


    def __str__(self):
        return str(self.particles)
