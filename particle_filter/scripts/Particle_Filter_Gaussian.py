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

        self.history=np.array((0,0,0,0))
        #self.history=np.array((self.x[0],self.y[0],self.z[0],init_conc))
        self.sens_number = sens_number

    def reading(self, gas_conc):
        '''
        Records Gas Concentration at sensor location
        '''
        self.gas_conc = gas_conc
        self.history = np.append(self.history,((self.x,self.y,self.z,self.gas_conc))) # keep track of where we have been

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

        self.x = x
        self.y = y
        self.z = z

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

        def pdf_thresh_function(sensorval):
            # if sensorval < 2:
            #     return 100
            # elif sensorval <10:
            #     return 35
            # elif sensorval < 100:
            #     return 25
            # elif sensorval <200:
            #     return 20
            # else:
            #     return 200
            return 500



        x_world = self.particles[0]
        y_world = self.particles[1]
        z_world = self.particles[2]
        theta = self.particles[3]
        Q = self.particles[4]
        V = self.particles[5]
        Dy = self.particles[6]
        Dz = self.particles[7]

        particle_likelihoods = np.zeros((self.num_of_parti))
        #print(particle_likelihoods)

        # function for pdf threshold
        pdf_std = pdf_thresh_function(actual_sensor_reading)

        # x_world[0] = 25
        # y_world[0] = 45
        # z_world[0] = 2
        # theta[0] = (6*np.pi)/4
        # Q[0] = 100000
        # V[0] = 4
        # Dy[0] = 40
        # Dz[0] = .01
        # print('sensor x')
        # print(sensor_x)
        # print('sensor y')
        # print(sensor_y)
        # print('sensor z')
        # print(sensor_z)

        #print(pdf_std)
        # converts quad to particle plume space and calculate the would be concentration at that point
        for d in range(0,self.num_of_parti):
            Xp = np.cos(theta[d])*sensor_x + np.sin(theta[d])*sensor_y + (-np.cos(theta[d])*x_world[d] - np.sin(theta[d])*y_world[d])
            Yp = -np.sin(theta[d])*sensor_x + np.cos(theta[d])*sensor_y + (np.sin(theta[d])*x_world[d] - np.cos(theta[d])*y_world[d])
            Zp = sensor_z - z_world[d]

            if Xp<=0:
                conc = 0
            else:
                conc = (Q[d]/((4*np.pi*Xp)*np.sqrt(Dy[d]*Dz[d])))*(np.exp((-V[d]/(4*Xp))*((Yp**2/Dy[d])+(Zp**2/Dz[d]))))
                # if d == 0:
                #     print('real center value conc:')
                #     print(conc)
                #     print('xp:')
                #     print(Xp)
                #     print('yp:')
                #     print(Yp)




            particle_likelihood = norm.pdf(conc, actual_sensor_reading, pdf_std)
            particle_likelihood = particle_likelihood/(norm.pdf(0,0, pdf_std))
            #print(particle_likelihood[0])
            particle_likelihoods[d] = particle_likelihood


        # finds out what particles have been placed outside vineyard and makes them unlikely
        out_area_particles_x = (self.particles[0] <= self.vin_size[0])*(self.particles[0] >= 0)
        out_area_particles_y = (self.particles[1] <= self.vin_size[1])*(self.particles[1] >= 0)
        particle_likelihoods = particle_likelihoods*out_area_particles_x
        particle_likelihoods = particle_likelihoods*out_area_particles_y
        #print(particle_likelihoods)

        prob_df = particle_likelihoods/sum(particle_likelihoods)
        prob_df = prob_df*self.prior
        prob_df = prob_df/sum(prob_df)
        self.prob_df = prob_df

        # find particles that have rectangle spaces outside of field and make them unlikely....
        #sense_likehood = sense_likehood*((self.particles[0]+rect_width_particles) < self.estimated_parameters['x_particles'][1]+5)
        #sense_likehood = sense_likehood*((self.particles[0]-rect_width_particles) > self.estimated_parameters['x_particles'][0]-5)
        #sense_likehood = sense_likehood*((self.particles[1]+rect_height_particles) < self.estimated_parameters['y_particles'][1]+5)
        #sense_likehood = sense_likehood*((self.particles[1]-rect_height_particles) > self.estimated_parameters['y_particles'][0]-5
        return

    def resamp_and_noise(self, noise_std_params=_NOISE_STD_PARAMS, num_of_parti=_NUM_OF_PARTICLES, impov_particles = _IMP_PARTICLES):
        '''
        Generates new particles that are based on the likelihood of the current ones
        '''
        particle_indices = range(0,num_of_parti,1)
        # print(num_of_parti-_IMP_PARTICLES)

        resampled_index = np.random.choice(particle_indices, 800, replace=True, p = self.prob_df)
        new_imp_particles = np.zeros((self.num_of_parameters,_IMP_PARTICLES))
        new_particles = np.zeros((self.num_of_parameters,num_of_parti-_IMP_PARTICLES))
        for x in range(len(self.particles)):
            noise = np.random.normal(0, noise_std_params[x], num_of_parti-_IMP_PARTICLES)
            new_particles[x] = abs(self.particles[x][resampled_index]+noise)
            new_imp_particles[x] = np.random.uniform(self.estimated_parameters[x][0],self.estimated_parameters[x][1],_IMP_PARTICLES)

        self.particles = np.concatenate((new_particles,new_imp_particles),axis=1)


        new_particles_prior = self.prob_df[resampled_index]
        new_imp_particles_prior = (1/_NUM_OF_PARTICLES)*np.ones((_IMP_PARTICLES,))
        self.prior = np.concatenate((new_particles_prior,new_imp_particles_prior))

        # generate noise and add it to new particles
        return


    def __str__(self):
        return str(self.particles)




# sensors = Sensor() # initilize sensors (starts at random location)
# particle_filter = Particle_Gen() # initializes particles

# for w in range(_NUM_OF_STEPS):
#     x = np.random.uniform(0,_VINYARD_SIZE[_X],1) # selects random x value for sensor
#     y = np.random.uniform(0,_VINYARD_SIZE[_Y],1) # selects random y value for sensor
#     sensors.move(x,y) # moves sensor to new location
#     particle_filter.likelihood(plume_params, sensors.reading(), sensors.x , sensors.y) # gets likelihod of each particle


#     # time.sleep(1)
#     particle_filter.resamp_and_noise() # resamples likely particles and adds noise to them
#     mpl.plot(particle_filter.particles[_X],particle_filter.particles[_Y],'.')
#     mpl.plot(sensors.x[0][0],sensors.y[0][0],'bo')
#     mpl.plot(sensors.history[0],sensors.history[1],'r.')
#     mpl.vlines(range(1,_VINYARD_SIZE[_X],_ROW_SPACEING),0,_VINYARD_SIZE[_Y],colors='g')
#     mpl.title('Sensor Reading ' + str(w+1))
#     #mpl.xlim(0,_VINYARD_SIZE[_X])
#     #mpl.ylim(0,_VINYARD_SIZE[_Y])
#     mpl.show()


# rect_parameters = np.mean(particle_filter.particles,axis=1)
# TL_X = rect_parameters[0]-(rect_parameters[2]/2)
# TL_Y = rect_parameters[1]-(rect_parameters[3]/2)

# fig, ax = mpl.subplots()
# ax.plot(Bugs_X_Locations,Bugs_Y_Locations,'g.')
# # ax.xlim(0,_VINYARD_SIZE[_X])
# # ax.ylim(0,_VINYARD_SIZE[_Y])
# rect = patch.Rectangle((TL_X,TL_Y),rect_parameters[2],rect_parameters[3],edgecolor='r', facecolor="none")
# ax.add_patch(rect)

# mpl.show()






#r = particles.hypothetical_readings(plume_params, sensors.x , sensors.y)


# while w[0]==0:
#     sensors = Sensor()
#     w = sensors.reading()
#     print(sensors.reading())
#     print(sensors.x)
#     print(sensors.y)
#     r = r+1
