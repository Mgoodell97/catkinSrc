# -*- coding: utf-8 -*-
"""
Created on Tue Oct 13 15:00:40 2020

@author: Nathan
"""
import time
import numpy as np
from scipy.stats import norm
import matplotlib.pyplot as mpl
import matplotlib.patches as patch

# First Create Sensors to Detect Bugs
_NUM_OF_SENSORS = 1
_VINYARD_SIZE = [100, 100]
_NUM_OF_PARTICLES = 1000
_PARTICLE_PARAMETERS = [[0, 50],[0, 50],[0,4],[0,6*np.pi/4],[50000, 50000],[1, 1],[.15,.15],[.15,.15]] #x,y,z,theta,Q,V,Dy,Dz
_IMP_PARTICLES = 200

_X_NOISE_STD = .1
_Y_NOISE_STD =  .5
_WIDTH_STD = .5
_HEIGHT_STD = .5
_NOISE_STD_PARAMS = np.array((_X_NOISE_STD,_Y_NOISE_STD,_WIDTH_STD,_HEIGHT_STD))
_NUM_OF_STEPS = 10
_ROW_SPACEING = 2

_X = 0
_Y = 1
_Z = 2
_PDF_STD_DIST = .35
#lume_params = [66.7853, 66.1059, 0.1372, 2.4559] #[Q,Dy,Dx,V]


class Sensor():
    '''
    Sensor class- places inital sensor(s) in random location(s)
    '''
    def __init__(self, sens_number=_NUM_OF_SENSORS, vin_size=_VINYARD_SIZE, x_loc = None, y_loc = None, z_loc = None):
        '''
        x - x-location
        y - y-location
        '''
        if x_loc == None:
            x = np.random.uniform(0,vin_size[_X],sens_number)
            self.x = np.reshape(x,(sens_number,1))
        else:
            self.x = np.reshape(x_loc,(sens_number,1))

        if y_loc == None:
            y = np.random.uniform(0,vin_size[_Y],sens_number)
            self.y = np.reshape(y,(sens_number,1))
        else:
            self.y = np.reshape(y_loc,(sens_number,1))

        if z_loc == None:
            z = np.random.uniform(0,vin_size[_Z],sens_number)
            self.z = np.reshape(z,(sens_number,1))
        else:
            self.z = np.reshape(z_loc,(sens_number,1))

        self.history=np.array([])
        self.xhistory = np.array([])
        self.yhistory = np.array([])
        self.zhistory = np.array([])
        #self.history=np.array((self.x[0],self.y[0],self.z[0],init_conc))
        self.sens_number = sens_number

    def reading(self, gas_conc, x, y, z):
        '''
        Records Gas Concentration at sensor location
        '''
        if gas_conc>0:
            self.history = np.append(self.history,gas_conc) # keep track of where we have been
            self.xhistory = np.append(self.xhistory,x) # keep track of where we have been
            self.yhistory = np.append(self.yhistory,y) # keep track of where we have been
            self.zhistory = np.append(self.zhistory,z) # keep track of where we have been

            # mpl.ion()
            # mpl.show()
            # mpl.clf()
            # mpl.scatter(self.xhistory,self.yhistory,c=self.history ,cmap='jet')
            # mpl.colorbar()
            # mpl.xlim(0,50)
            # mpl.ylim(0,50)
            # mpl.xlabel('X')
            # mpl.ylabel('Y')
            # mpl.title('Gaden Concentrations at XY Locations')
            # mpl.draw()
            # mpl.pause(.5)



    def move(self, x=-1, y=-1, z=-1):
        '''
        move takes in 2 np.array() values that have the same size as the current sensor
        for 1 -> n sensors
        x = np.array([[X1],[X2],[Xn]])
        y = np.array([[Y1],[Y2],[Yn]])
        Later I could add typs of movement inputs instead???

        Raster Scan = X=-1, Y=-1
        Random Scan = X=-2, Y=-2
        '''

        #self.history = np.append(self.history,((x,y,z,self.gas_conc))) # keep track of where we have been

        x = np.reshape(x,(self.sens_number,1))
        y = np.reshape(y,(self.sens_number,1))
        z = np.reshape(z,(self.sens_number,1))

        if np.size(self.x) != np.size(x):
            print('need same number of movements as sensor')
            return
        if np.size(self.y) != np.size(y):
            print('need same number of movements as sensor')
            return
        if np.size(self.z) != np.size(z):
            print('need same number of movements as sensor')
            return

        self.x = x[0][0]
        self.y = y[0][0]
        self.z = z[0][0]

class GaussianPlume(): # not needed in ROS I already have this from matthews simulation
    def __init__(self, Dy=66.1059, Dx=0.1372, V=2.4559, Q=66.7853):
        self.Dy = Dy
        self.Dx = Dx
        self.V = V
        self.Q = Q

    def plume_conc(self, x_world, y_world, theta=0):
        self.x_world = x_world
        self.y_world = y_world
        self.conc = self.conc
        return 'NOT DONE YET'

class Particle_Gen():
    def __init__(self, estimated_parameters=_PARTICLE_PARAMETERS, num_of_parti=_NUM_OF_PARTICLES, sens_number = _NUM_OF_SENSORS, vin_size=_VINYARD_SIZE):
        self.estimated_parameters = estimated_parameters
        #print(estimated_parameters)
        self.num_of_parti = num_of_parti
        self.num_of_parameters = len(estimated_parameters)
        self.particles = np.zeros((self.num_of_parameters,self.num_of_parti))
        self.prior = np.ones((num_of_parti))
        self.sens_number = sens_number
        self.vin_size = vin_size
        self.good_readings = [0]
        self.good_readings_x = [0]
        self.good_readings_y = [0]
        self.good_readings_z = [0]

        idx = 0

        for x in range(0,self.num_of_parameters):
            #print(x)
            self.particles[idx] = np.random.uniform(estimated_parameters[x][0],estimated_parameters[x][1],num_of_parti)
            idx += 1

    def likelihood(self, actual_sensor_reading, sensor_x, sensor_y, sensor_z):
        '''
        Gets what sensor reading would be at the same location of the sensor
        within the plume of each particle
        '''
        x_world = self.particles[0]
        y_world = self.particles[1]
        z_world = self.particles[2]
        theta = self.particles[3]
        Q = self.particles[4]
        V = self.particles[5]
        Dy = self.particles[6]
        Dz = self.particles[7]

        print(actual_sensor_reading)
        # def pdf_thresh_function(sensorval):
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

        def pdf_thresh_function(sensorval):
            return 500

        pdf_std = pdf_thresh_function(actual_sensor_reading)


        if actual_sensor_reading>8000000000:
            if self.good_readings[0] == 0:
                self.good_readings.pop(0)
                self.good_readings_x.pop(0)
                self.good_readings_y.pop(0)
                self.good_readings_z.pop(0)
            self.good_readings.append(actual_sensor_reading)
            self.good_readings_x.append(sensor_x)
            self.good_readings_y.append(sensor_y)
            self.good_readings_z.append(sensor_z)
            time.sleep(10)
            concentrations = np.zeros((len(self.good_readings),self.num_of_parti))        #print(particle_likelihoods)
            particle_likelihoods = np.zeros((len(self.good_readings),self.num_of_parti))
            Xp = np.zeros((len(self.good_readings),self.num_of_parti))
            Yp = np.zeros((len(self.good_readings),self.num_of_parti))
            Zp = np.zeros((len(self.good_readings),self.num_of_parti))

            for past_positives in range(0,len(self.good_readings)):
                Xp[past_positives] = np.cos(theta)*self.good_readings_x[past_positives] + np.sin(theta)*self.good_readings_y[past_positives] + (-np.cos(theta)*x_world - np.sin(theta)*y_world)
                Yp[past_positives] = -np.sin(theta)*self.good_readings_x[past_positives] + np.cos(theta)*self.good_readings_y[past_positives] + (np.sin(theta)*x_world - np.cos(theta)*y_world)
                Zp[past_positives] = self.good_readings_z[past_positives] - z_world

            for past_positives in range(0,len(self.good_readings)):
                xp_bad = np.where(Xp[past_positives]<=0)

                xp_good = np.where(Xp[past_positives]>0)
                xp_bad = xp_bad[0]
                xp_good = xp_good[0]

                concentrations[past_positives,xp_bad] = 0
                concentrations[past_positives,xp_good] = (Q[xp_good]/((4*np.pi*Xp[past_positives,xp_good])*np.sqrt(Dy[xp_good]*Dz[xp_good])))*(np.exp((-V[xp_good]/(4*Xp[past_positives,xp_good]))*((Yp[past_positives,xp_good]**2/Dy[xp_good])+(Zp[past_positives,xp_good]**2/Dz[xp_good]))))
                #time.sleep(.5)
                particle_likelihoods[past_positives] = norm.pdf(concentrations[past_positives],self.good_readings[past_positives],pdf_std)
                particle_likelihoods[past_positives] = particle_likelihoods[past_positives]/(norm.pdf(0,0,pdf_std))
                out_area_particles_x = (self.particles[0] <= self.vin_size[0])*(self.particles[0] >= 0)
                out_area_particles_y = (self.particles[1] <= self.vin_size[1])*(self.particles[1] >= 0)
                particle_likelihoods[past_positives] = particle_likelihoods[past_positives]*out_area_particles_x
                particle_likelihoods[past_positives] = particle_likelihoods[past_positives]*out_area_particles_y

            particle_likelihoods = np.prod(particle_likelihoods,0)
            #print(particle_likelihoods)

            if sum(particle_likelihoods) == 0:
                print('NaN Error!')
                    #print(actual_sensor_reading)
                    # error = actual_sensor_reading-concentrations
                    # mpl.ion()
                    # mpl.show()
                    # mpl.clf()
                    # mpl.hist(concentrations,bins=100)
                    # mpl.draw()
                    # mpl.pause(.5)

            else:
                error = actual_sensor_reading-concentrations
                    # mpl.ion()
                    # mpl.show()
                    # mpl.clf()
                    # mpl.hist(error,range=(-10000,10000),bins=100)
                    # mpl.draw()
                    # mpl.pause(.5)
                    # print('TESTING:')
                    # print(actual_sensor_reading)
                prob_df = particle_likelihoods/sum(particle_likelihoods)
                prob_df = prob_df*self.prior
                prob_df = prob_df/sum(prob_df)
                self.prob_df = prob_df

            return






        else:
            concentrations = np.zeros((self.num_of_parti))        #print(particle_likelihoods)
            particle_likelihoods = np.zeros((self.num_of_parti))
            Xp = np.cos(theta)*sensor_x + np.sin(theta)*sensor_y + (-np.cos(theta)*x_world - np.sin(theta)*y_world)
            Yp = -np.sin(theta)*sensor_x + np.cos(theta)*sensor_y + (np.sin(theta)*x_world - np.cos(theta)*y_world)
            Zp = sensor_z - z_world

            xp_bad = np.where(Xp<=0)
            xp_good = np.where(Xp>0)

            xp_bad = xp_bad[0]
            xp_good = xp_good[0]


            concentrations[xp_bad] = 0
            concentrations[xp_good] = (Q[xp_good]/((4*np.pi*Xp[xp_good])*np.sqrt(Dy[xp_good]*Dz[xp_good])))*(np.exp((-V[xp_good]/(4*Xp[xp_good]))*((Yp[xp_good]**2/Dy[xp_good])+(Zp[xp_good]**2/Dz[xp_good]))))

            particle_likelihoods = norm.pdf(concentrations,actual_sensor_reading,pdf_std)
            particle_likelihoods = particle_likelihoods/(norm.pdf(0,0,pdf_std))


            # finds out what particles have been placed outside vineyard and makes them unlikely
            out_area_particles_x = (self.particles[0] <= self.vin_size[0])*(self.particles[0] >= 0)
            out_area_particles_y = (self.particles[1] <= self.vin_size[1])*(self.particles[1] >= 0)
            particle_likelihoods = particle_likelihoods*out_area_particles_x
            particle_likelihoods = particle_likelihoods*out_area_particles_y

            if sum(particle_likelihoods) == 0:
                print('NaN Error!')
                #print(actual_sensor_reading)
                # error = actual_sensor_reading-concentrations
                # mpl.ion()
                # mpl.show()
                # mpl.clf()
                # mpl.hist(concentrations,bins=100)
                # mpl.draw()
                # mpl.pause(.5)

            else:
                error = actual_sensor_reading-concentrations
                # mpl.ion()
                # mpl.show()
                # mpl.clf()
                # mpl.hist(error,range=(-10000,10000),bins=100)
                # mpl.draw()
                # mpl.pause(.5)
                # print('TESTING:')
                # print(actual_sensor_reading)
                prob_df = particle_likelihoods/sum(particle_likelihoods)
                prob_df = prob_df*self.prior
                prob_df = prob_df/sum(prob_df)
                if sum(prob_df) > 0:
                    self.prob_df = prob_df

            return



    def resamp_and_noise(self, noise_std_params=_NOISE_STD_PARAMS, num_of_parti=_NUM_OF_PARTICLES, impov_particles = _IMP_PARTICLES):
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


        new_particles_prior = self.prob_df[resampled_index]
        new_imp_particles_prior = (1/num_of_parti)*np.ones((impov_particles,))
        self.prior = np.concatenate((new_particles_prior,new_imp_particles_prior))

        # generate noise and add it to new particles
        return


    def __str__(self):
        return str(self.particles)
