#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
from particle_filter.msg import particles

import numpy as np
from math import pi

def main():

    rospy.init_node('Particle_Filter_Fake_Publisher') # name your node
    ## Initialization of Particles and Sensor ##

    rate = rospy.Rate(5)

    particlesMsg = particles()
    particlesPublisher = rospy.Publisher('fake_particles', particles, queue_size=5)

    while not rospy.is_shutdown(): # while roscore is running do this. if not, stop... Executes Function
        # Generate particles : theta, x, y, x, Q, v, Dy, Dz
        xp = np.array([[0, 25, 50, 0, 40000, 2, 7, 0.5], [pi, 75, 50, 0, 40000, 3, 7, 0.5], [pi/2, 40, 25, 0, 40000, 3, 7, 0.5], [pi/4, 50, 50, 0, 40000, 3, 7, 0.5], [-pi/2, 80, 75, 0, 40000, 3, 7, 0.5]])  # theta, x, y, x, Q, v, Dy, Dz
        wp = np.array([0.2, 0.2, 0.2, 0.2, 0.2])

        particlesMsg.theta   = xp[:,0]
        particlesMsg.X       = xp[:,1]
        particlesMsg.Y       = xp[:,2]
        particlesMsg.Z       = xp[:,3]
        particlesMsg.Q       = xp[:,4]
        particlesMsg.v       = xp[:,5]
        particlesMsg.Dy      = xp[:,6]
        particlesMsg.Dz      = xp[:,7]
        particlesMsg.weights = wp

        particlesPublisher.publish(particlesMsg)



        rate.sleep() # have you met the rospy.Rate? if not wait some amount of time.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
