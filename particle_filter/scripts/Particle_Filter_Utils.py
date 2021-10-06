# -*- coding: utf-8 -*-
"""
Created on Tue Oct 13 15:00:40 2020

@author: Nathan
"""
import time
import numpy as np
from numpy import inf
import math
from scipy.stats import norm
from scipy.stats import poisson

import matplotlib.pyplot as mpl
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
        self.xhistory = np.array([])
        self.yhistory = np.array([])
        self.zhistory = np.array([])
        self.sens_number = sens_number
        self.max_conc = 0
        self.max_conc_loc = [0,0,0]
    def reading(self, gas_conc, x, y, z):
        '''
        Records Gas Concentration at sensor location
        '''
        self.history = np.append(self.history,gas_conc) # keep track of where we have been
        self.xhistory = np.append(self.xhistory,x) # keep track of where we have been
        self.yhistory = np.append(self.yhistory,y) # keep track of where we have been
        self.zhistory = np.append(self.zhistory,z) # keep track of where we have been

        # if len(self.history) == 100:
        #     print(self.history)
        #     print(self.xhistory)
        #     time.sleep(10)

        if gas_conc>=self.max_conc:
            self.max_conc = gas_conc
            self.max_conc_loc = np.array([x,y,z])

class Particle_Gen():
    def __init__(self, estimated_parameters, num_of_parti, sens_number, pdf_std):
        self.estimated_parameters = estimated_parameters
        self.num_of_parti = num_of_parti
        self.num_of_parameters = len(estimated_parameters)
        self.particles = np.zeros((self.num_of_parameters,self.num_of_parti))
        self.prior = np.ones((num_of_parti))
        self.sens_number = sens_number
        self.prob_df = np.ones((num_of_parti))/num_of_parti
        self.pdf_std = pdf_std

        idx = 0
        for x in range(0,self.num_of_parameters):
            #print(x)
            self.particles[idx] = np.random.uniform(estimated_parameters[x][0],estimated_parameters[x][1],num_of_parti)
            idx += 1

        xStd     = np.std(self.particles[0])
        yStd     = np.std(self.particles[1])
        thetaStd = np.std(self.particles[3])

        stdVec = np.array([xStd, yStd, thetaStd])

        self.stdL2NormMax = np.linalg.norm(stdVec)


    def normalize(self):
        self.prob_df = self.prob_df/sum(self.prob_df)

    def particle_removal(self):
        out_area_particles = np.ones((self.num_of_parti))

        for i in range(self.num_of_parameters):
            # print(i)
            # print((self.particles[i] <= self.estimated_parameters[i][1]))
            # print((self.particles[i] >= self.estimated_parameters[i][0]))
            out_area_particles = out_area_particles*((self.particles[i] <= self.estimated_parameters[i][1])*(self.particles[i] >= self.estimated_parameters[i][0]))
        return out_area_particles

    def pdf_thresh_function(self, sensorval):
        new_pdf_std = np.log(sensorval+1)
        return new_pdf_std


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


    def likelihood(self, actual_sensor_readings, sensor_xs, sensor_ys, sensor_zs, max_conc_quad):#, UAV1_maxconc, UAV2_maxconc, UAV3_maxconc):
        '''
        Gets what sensor reading would be at the same location of the sensor
        within the plume of each particle
        '''
        x_world = self.particles[0].reshape(self.num_of_parti,1)
        y_world = self.particles[1].reshape(self.num_of_parti,1)
        z_world = self.particles[2].reshape(self.num_of_parti,1)
        theta = self.particles[3].reshape(self.num_of_parti,1)
        Q = self.particles[4].reshape(self.num_of_parti,1)
        V = self.particles[5].reshape(self.num_of_parti,1)
        Dy = self.particles[6].reshape(self.num_of_parti,1)
        Dz = self.particles[7].reshape(self.num_of_parti,1)

        # actual_sensor_readings = np.log(actual_sensor_readings+1)/np.log(10)

        # pdf_std = self.pdf_thresh_function(actual_sensor_readings)

        # pdf_std = 0.225 good for 50x50 sims

        particle_likelihoods = np.zeros((self.num_of_parti))

        Xp = np.matmul(np.cos(theta),sensor_xs) + np.matmul(np.sin(theta),sensor_ys) + (-np.cos(theta)*x_world - np.sin(theta)*y_world)
        Yp = np.matmul(-np.sin(theta),sensor_xs) + np.matmul(np.cos(theta),sensor_ys) + (np.sin(theta)*x_world - np.cos(theta)*y_world)
        Zp = sensor_zs - z_world

        concentrations = np.zeros(Xp.shape)        #print(particle_likelihoods)

        concentrations = ((Q/(4*np.pi*Xp*np.sqrt(Dy*Dz)))*(np.exp( (-V/(4*Xp)) * ( (Yp**2/Dy) + (Zp**2/Dz) )) )) * 1000 # convert from kg/m^3 to ppm


        concentrations[concentrations == inf] = 50000
        concentrations[concentrations < 0] = 0

        xStd     = np.std(self.particles[0])
        yStd     = np.std(self.particles[1])
        thetaStd = np.std(self.particles[3])

        stdVec = np.array([xStd, yStd, thetaStd])

        # print(stdVec)

        stdL2Norm = np.linalg.norm(stdVec)

        print(stdL2Norm)

        if self.stdL2NormMax <= stdL2Norm:
            self.stdL2NormMax = stdL2Norm

        r = self.pdf_std * stdL2Norm / self.stdL2NormMax

        particle_likelihoods = norm.pdf(actual_sensor_readings,concentrations,self.pdf_std)
        particle_likelihoods = particle_likelihoods.prod(1)

        # # print(particle_likelihoods)
        # # print(particle_likelihoods.shape)
        #
        # particle_likelihoods = particle_likelihoods*best_conc_like#/(norm.pdf(0,0,pdf_std))

        # print(concentrations[np.argmax(particle_likelihoods)])

        # finds out what particles have been placed outside vineyard and makes them unlikely
        # out_area_particles = self.particle_removal()
        # particle_likelihoods = particle_likelihoods*out_area_particles

        # print(particle_likelihoods)

        if sum(particle_likelihoods) == 0:
            print('NaN Error!')
        else:
            prob_df = particle_likelihoods/sum(particle_likelihoods)
            prob_df = prob_df*self.prior
            prob_df = prob_df/sum(prob_df)
            if sum(prob_df) > 0:
                self.prob_df = prob_df
        return

    def likelihood_ratebased(self, actual_sensor_readings, sensor_xs, sensor_ys, sensor_zs, max_conc_quad, sampling_frequency=1):
        x_world = self.particles[0].reshape(self.num_of_parti,1)
        y_world = self.particles[1].reshape(self.num_of_parti,1)
        z_world = self.particles[2].reshape(self.num_of_parti,1)
        theta = self.particles[3].reshape(self.num_of_parti,1)
        Q = self.particles[4].reshape(self.num_of_parti,1)
        V = self.particles[5].reshape(self.num_of_parti,1)
        D = self.particles[6].reshape(self.num_of_parti,1)
        Tau = self.particles[7].reshape(self.num_of_parti,1)

        actual_sensor_readings = np.log(actual_sensor_readings+1)/np.log(10)

        gamma = np.sqrt((D*Tau)/(1+(((V**2)*Tau)/(4*D))))



        Xp = np.matmul(np.cos(theta),sensor_xs) + np.matmul(np.sin(theta),sensor_ys) + (-np.cos(theta)*x_world - np.sin(theta)*y_world)
        Yp = np.matmul(-np.sin(theta),sensor_xs) + np.matmul(np.cos(theta),sensor_ys) + (np.sin(theta)*x_world - np.cos(theta)*y_world)
        Zp = sensor_zs - z_world


        X_l = np.stack([Xp,Yp,Zp],1)

        X_l_N = np.linalg.norm(X_l,axis=1)

        rates = ((Q)/(X_l_N))*np.exp(-((X_l_N/gamma)+((Xp*V)/(2*D))))

        rates = np.log(rates+1)/np.log(10)
        # mpl.hist(rates)
        # mpl.show()
        # time.sleep(2)


        Z_k = np.round(actual_sensor_readings)
        thresh = 2000
        Z_k = np.where(Z_k > thresh, thresh, Z_k)


        Z_k_hat = np.round(rates)

        # print(max(Z_k_hat))
        fs = sampling_frequency



        pois_dist_likelihoods = poisson.pmf(Z_k,(1/fs)*Z_k_hat)
        pois_dist_likelihoods = pois_dist_likelihoods.prod(1)
        # print(pois_dist_likelihoods.shape)

        X_k = np.array([max_conc_quad.X,max_conc_quad.Y,max_conc_quad.Z])
        X_s = np.stack([x_world,y_world,z_world],2)
        # pdf_std = self.pdf_thresh_function(actual_sensor_readings)


        norm_dist_likelihoods = norm.pdf(X_k,X_s,100)
        norm_dist_likelihoods = norm_dist_likelihoods.prod(2)[:,0]
        # print(norm_dist_likelihoods.shape)

        # print(norm_dist_likelihoods)
        # time.sleep(5)

        # likelihood_joined = norm_dist_likelihoods
        # likelihood_joined = pois_dist_likelihoods
        if max_conc_quad.C > 0:
            likelihood_joined = pois_dist_likelihoods*norm_dist_likelihoods
        else:
            likelihood_joined = pois_dist_likelihoods
        # print(likelihood_joined)
        # time.sleep(5)

        # finds out what particles have been placed outside vineyard and makes them unlikely
        out_area_particles = self.particle_removal()
        likelihood_joined = likelihood_joined*out_area_particles

        if sum(likelihood_joined) == 0:
            print('NaN Error!')
        else:
            # print('Success!!')
            prob_df = likelihood_joined/sum(likelihood_joined)
            prob_df = prob_df*self.prior
            prob_df = prob_df/sum(prob_df)
            if sum(prob_df) > 0:
                self.prob_df = prob_df
        return

    def resample(self, num_of_parti, impov_particles):
        '''
        Generates new particles that are based on the likelihood of the current ones
        '''
        particle_indices = range(0,num_of_parti,1)
        # print(num_of_parti-_IMP_PARTICLES)

        resampled_index = np.random.choice(particle_indices, num_of_parti-impov_particles, replace=True, p = self.prob_df)
        new_imp_particles = np.zeros((self.num_of_parameters,impov_particles))
        new_particles = np.zeros((self.num_of_parameters,num_of_parti-impov_particles))
        for x in range(len(self.particles)):
            new_particles[x] = abs(self.particles[x][resampled_index])
            new_imp_particles[x] = np.random.uniform(self.estimated_parameters[x][0],self.estimated_parameters[x][1],impov_particles)

        self.particles = np.concatenate((new_particles,new_imp_particles),axis=1)

        # keeps around old prior??? I think it starts uniform after resampling???
        #new_particles_prior = self.prob_df[resampled_index]
        #new_imp_particles_prior = (1/num_of_parti)*np.ones((impov_particles,))
        self.prior = np.ones((num_of_parti))

        return

    def state_transition(self, noise_std_params):
        for x in range(len(self.particles)):
            noise = np.random.normal(0, noise_std_params[x], self.num_of_parti)
            self.particles[x] = abs(self.particles[x]+noise)
        return

    def LLE(self, desired_number):
        idx = np.argpartition(self.prob_df, desired_number)
        least_likely_particles = self.particles[:,idx[:desired_number]]
        return least_likely_particles, idx

    def MLE(self, desired_number):
        idx = np.argpartition(self.prob_df, -desired_number)
        most_likely_particles = self.particles[:,idx[-desired_number:]]
        return most_likely_particles, idx

    def resamp_and_noise(self, noise_std_params, num_of_parti, impov_particles):
        '''
        Generates new particles that are based on the likelihood of the current ones
        '''
        particle_indices = range(0,num_of_parti,1)
        # print(num_of_parti-_IMP_PARTICLES)

        resampled_index = np.random.choice(particle_indices, num_of_parti-impov_particles, replace=True, p = self.prob_df)
        new_imp_particles = np.zeros((self.num_of_parameters,impov_particles))
        new_particles = np.zeros((self.num_of_parameters,num_of_parti-impov_particles))
        for x in range(len(self.particles)):
            noise = np.random.normal(0, noise_std_params[x], num_of_parti-impov_particles)
            new_particles[x] = abs(self.particles[x][resampled_index]+noise)
            new_imp_particles[x] = np.random.uniform(self.estimated_parameters[x][0],self.estimated_parameters[x][1],impov_particles)

        self.particles = np.concatenate((new_particles,new_imp_particles),axis=1)


        # new_particles_prior = self.prob_df[resampled_index]
        # new_imp_particles_prior = (1/num_of_parti)*np.ones((impov_particles,))
        self.prior = np.ones((num_of_parti))

        # generate noise and add it to new particles
        return


    def __str__(self):
        return str(self.particles)
