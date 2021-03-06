#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
#import numpy
from Particle_Filter_Gaussian import *
# from sensor_msgs.msg import Joy
from olfaction_msgs.msg import gas_sensor
# import sys


global chem_reading
_NUM_OF_PARTICLES = 10000
_PARTICLE_PARAMETERS = {'x': [0, 14], 'y': [0, 14], 'z': [0,7], 'Theta': [0, 2*np.pi],'Q': [0.1, 0.25], 'V': [4, 10], 'Dy': [40,65], 'Dz': [0, 1]}
_NUM_OF_ROBOTS = 1
_VINYARD_SIZE = [100, 100]
_PDF_STD_DIST = .35

x_noise = .5
y_noise = .5
z_noise = .5
theta_noise = .1
Q_noise = .01
V_noise = .1
Dy_noise = .1
Dz_noise = .01

_NOISE_STD_PARAMS = np.array((x_noise,y_noise,z_noise,theta_noise,Q_noise,V_noise,Dy_noise,Dz_noise))


chem_reading = gas_sensor()
# controllerValues.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# controllerValues.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def chem_sens_cb(raw_readings_msg): # function(variable to store message data)
    global chem_reading
    chem_reading = raw_readings_msg

def initFunc():
    rospy.init_node('Particle_Filter') # name your node

    rospy.Subscriber("/PID00/Sensor_reading", gas_sensor, chem_sens_cb) # subscribe to nodes... rospy.Subscriber(topic of interest, mesage type of interest, function that you are going to run)

    global rate # set up refresh rate
    rate = rospy.Rate(5) # publish at rate of 20 Hz


def main():
    initFunc() # sets up function
    ## Initialization of Particles and Sensor ##
    particle_filter = Particle_Gen(_PARTICLE_PARAMETERS,_NUM_OF_PARTICLES,_NUM_OF_ROBOTS) # initializes particles
    robotXLoc = 7
    robotYLoc = 11
    robotZLoc = 2
    sensors = Sensor(_NUM_OF_ROBOTS,_VINYARD_SIZE,robotXLoc,robotYLoc,robotZLoc) # initilize sensors (starts at random location)
    print('Initializing...')

    while not rospy.is_shutdown(): # while roscore is running do this. if not, stop... Executes Function
        #rospy.loginfo("Throttle: %f     Roll: %f        Pitch: %f",controllerValues.axes[1],-controllerValues.axes[3],controllerValues.axes[4])
        global chem_reading
        #if chem_reading.raw > 5:
        # print(chem_reading.raw)
        print('getting chem_reading')
        print('chem_reading:')
        print(chem_reading.raw)
        sensors.reading(chem_reading.raw)
        particle_filter.likelihood(sensors.gas_conc, sensors.x , sensors.y, sensors.z, _PDF_STD_DIST) # gets likelihod of each particle
        particle_filter.resamp_and_noise(_NOISE_STD_PARAMS,_NUM_OF_PARTICLES) # resamples likely particles and adds noise to them
        print('test')
        print('X:')
        print(np.mean(particle_filter.particles[0]))
        print('Y:')
        print(np.mean(particle_filter.particles[1]))
        print('Z:')
        print(np.mean(particle_filter.particles[2]))

        # assume x and y is known later we will subscribe to topic matthew made

        rate.sleep() # have you met the rospy.Rate? if not wait some amount of time.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
