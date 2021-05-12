#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
import numpy as np
from Particle_Filter_Gaussian import *
import matplotlib.pyplot as plt
# from sensor_msgs.msg import Joy

# from message import message type
#from olfaction_msgs.msg import gas_sensor

from quadnodes.msg import gaussian
from olfaction_msgs.msg import gas_sensor
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles


#rom true_position.msg import
# import sys

global chem_reading
_NUM_OF_PARTICLES_GAUS = 1000
_NUM_OF_PARTICLES_GADEN = _NUM_OF_PARTICLES_GAUS

_IMP_PARTICLES = 100
_PARTICLE_PARAMETERS_GAUS = [[0,50],[0, 50],[2,2],[0,-2*np.pi],[50000, 50000],[1, 1],[.15,.15],[.15,.15]] #x,y,z,theta,Q,V,Dy,Dz
# _PARTICLE_PARAMETERS_GADEN = [[0,50],[0, 50],[2,2],[0,-2*np.pi],[25000, 75000],[.75, 1.25],[1.03697,1.03697],[.00007265,.00007265]] #x,y,z,theta,Q,V,Dy,Dz

#_PARTICLE_PARAMETERS = [[0,50],[0, 50],[2,2],[3*np.pi/2,3*np.pi/2],[50000, 50000],[1, 1],[.15,.15],[.15,.15]] #x,y,z,theta,Q,V,Dy,Dz
# _PARTICLE_PARAMETERS_GADEN = [[25,25],[45,45],[2,2],[3*np.pi/2,3*np.pi/2],[0, 50000],[0, 2.5],[0,1.5],[0,1.5]] #x,y,z,theta,Q,V,Dy,Dz

_PARTICLE_PARAMETERS_GADEN = [[0,50],[0,50],[2,2],[3*np.pi/2,3*np.pi/2],[0.01, 25],[25000, 75000],[0.1,5],[0.1,10]] #x,y,z,theta,Q,V,Dy,Dz

# _PARTICLE_PARAMETERS = [[0, 50],[0, 50],[0,4],[0,6*np.pi/4],[90000, 110000],[0, 8],[20,60],[.005, .025]]

_NUM_OF_ROBOTS = 1
_VINYARD_SIZE = [50, 50]
#_PDF_STD_DIST = 75

x_noise = 1
y_noise = 1
z_noise = 0
theta_noise = 0
Q_noise = 1000
V_noise = 0.02
Dy_noise = 0.025
Dz_noise = 0.025

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

_NOISE_STD_PARAMS = np.array((x_noise,y_noise,z_noise,theta_noise,Q_noise,V_noise,Dy_noise,Dz_noise))


# chem_reading = gaussian()
location_gauss = PoseStamped()
location_gaden = Marker()
plumeMsg = estimatedGaussian()
current_reading_full_data_gauss = gaussian()
current_reading_full_data_gaden = gas_sensor()
# controllerValues.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# controllerValues.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

# def chem_sens_cb(raw_readings_msg): # function(variable to store message data)
#     global chem_reading
#     chem_reading = raw_readings_msg

def gaussSensor_cb(gaussMsg):
    global current_reading_full_data_gauss
    current_reading_full_data_gauss = gaussMsg

def gadenSensor_cb(gadenMsg):
    global current_reading_full_data_gaden
    current_reading_full_data_gaden = gadenMsg

def location_gauss_get(loc_read_all): # function(variable to store message data)
    global location_gauss
    location_gauss = loc_read_all

def location_gaden_get(loc_read_gaden):
    global location_gaden
    location_gaden = loc_read_gaden

def main():
    rospy.init_node('Particle_Filter') # name your node
    ## Initialization of Particles and Sensor ##

    rate = rospy.Rate(5)

    PlumeType = rospy.get_param("particleFilter/PlumeType")      #  I'm not explaining this


    if PlumeType == "gaussian":
        rospy.Subscriber("gaussianReading", gaussian, gaussSensor_cb)
        rospy.Subscriber("true_position", PoseStamped, location_gauss_get)

        particle_filter = Particle_Gen(_PARTICLE_PARAMETERS_GAUS,_NUM_OF_PARTICLES_GAUS,_NUM_OF_ROBOTS,_VINYARD_SIZE) # initializes particles
    if PlumeType == "gaden":
        rospy.Subscriber("Sensor_reading", gas_sensor, gadenSensor_cb)
        rospy.Subscriber("Sensor_display", Marker, location_gaden_get)

        particle_filter = Particle_Gen(_PARTICLE_PARAMETERS_GADEN,_NUM_OF_PARTICLES_GADEN,_NUM_OF_ROBOTS,_VINYARD_SIZE) # initializes particles


    particlesMsg = particles()
    particlesPublisher = rospy.Publisher('particles', particles, queue_size=5)
    pub = rospy.Publisher("gaussianEstimation", estimatedGaussian, queue_size=10)



    X = 1
    Y = 1
    Z = 1
    Theta = 1
    Q = 1
    V = 1
    Dy = 1
    Dz = 1
    robotXLoc = 0
    robotYLoc = 0
    robotZLoc = 0
    sensors = Sensor(_NUM_OF_ROBOTS,_VINYARD_SIZE,robotXLoc,robotYLoc,robotZLoc) # initilize sensors (starts at random location)

    time.sleep(5)

    # print('Initializing...')
    while not rospy.is_shutdown(): # while roscore is running do this. if not, stop... Executes Function
        #rospy.loginfo("Throttle: %f     Roll: %f        Pitch: %f",controllerValues.axes[1],-controllerValues.axes[3],controllerValues.axes[4])
        global location_reading
        if PlumeType == "gaussian":
            chem_reading = current_reading_full_data_gauss.ppm
            x_location = location_gauss.pose.position.x
            y_location = location_gauss.pose.position.y
            z_location = location_gauss.pose.position.z
            sensors.reading(chem_reading,x_location,y_location,z_location)
            if chem_reading >= 0:
                particle_filter.likelihood(chem_reading, x_location , y_location, z_location) # gets likelihod of each particle
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

                particlesMsg.X = particle_filter.particles[0]
                particlesMsg.Y = particle_filter.particles[1]
                particlesMsg.Z = particle_filter.particles[2]
                particlesMsg.theta = particle_filter.particles[3]
                particlesMsg.Q = particle_filter.particles[4]
                particlesMsg.v = particle_filter.particles[5]
                particlesMsg.Dy = particle_filter.particles[6]
                particlesMsg.Dz = particle_filter.particles[7]
                particlesMsg.weights = particle_probabilities

                particlesPublisher.publish(particlesMsg)

                particle_filter.resamp_and_noise(_NOISE_STD_PARAMS,_NUM_OF_PARTICLES_GAUS, _IMP_PARTICLES) # resamples likely particles and adds noise to them

        if PlumeType == "gaden":
            chem_reading = current_reading_full_data_gaden.raw
            x_location = location_gaden.pose.position.x
            y_location = location_gaden.pose.position.y
            z_location = location_gaden.pose.position.z*2
            sensors.reading(chem_reading,x_location,y_location,z_location)

            if chem_reading > 10:
                particle_filter.likelihood(chem_reading, x_location , y_location, z_location) # gets likelihod of each particle
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

                print('Q:')
                print(Q)
                print('V:')

                print(V)
                print('Dy:')
                print(Dy)
                print('Dz:')
                print(Dz)


            # tab down
            particlesMsg.X = particle_filter.particles[0]
            particlesMsg.Y = particle_filter.particles[1]
            particlesMsg.Z = particle_filter.particles[2]
            particlesMsg.theta = particle_filter.particles[3]
            particlesMsg.Q = particle_filter.particles[4]
            particlesMsg.v = particle_filter.particles[5]
            particlesMsg.Dy = particle_filter.particles[6]
            particlesMsg.Dz = particle_filter.particles[7]
            try:
                particlesMsg.weights = particle_probabilities
            except:
                particlesMsg.weights = np.ones(_NUM_OF_PARTICLES_GAUS)/_NUM_OF_PARTICLES_GAUS

            particlesPublisher.publish(particlesMsg)

            if chem_reading > 10:
                particle_filter.resamp_and_noise(_NOISE_STD_PARAMS,_NUM_OF_PARTICLES_GADEN, _IMP_PARTICLES)

        plumeMsg.X = X
        plumeMsg.Y = Y
        plumeMsg.Z = Z
        plumeMsg.Theta = Theta
        plumeMsg.Q = Q
        plumeMsg.V = V
        plumeMsg.Dy = Dy
        plumeMsg.Dz = Dz

        pub.publish(plumeMsg)

        # assume x and y is known later we will subscribe to topic matthew made

        rate.sleep() # have you met the rospy.Rate? if not wait some amount of time.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
