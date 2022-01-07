#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
import tf_conversions
import tf
import tf2_ros
from visualization_msgs.msg import Marker
import numpy as np
import os, rospkg
import json
import pickle
from math import pi
import matplotlib.pyplot as plt

# Custom modules
from particleFilterPackage import ParticleFilter
from GaussianSensorPackage import combinePlumesNew, getReadingMultiPlume, accountForSensorDynamics

# Messages
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from olfaction_msgs.msg import gas_sensor
from mps_driver.msg import MPS

from particle_filter.msg import particles
from datetime import datetime

##################
# Global variables
##################

global sensorMsg_cb_flag
global ppm_reading
global x_t

sensorMsg_cb_flag = False;
ppm_reading = 0
x_t = np.zeros(3)

##################
# Functions
##################



##################
# Callbacks
##################



##################
# Main
##################

def main():

    rospy.init_node('infoPF')

    # Create meassages
    consumedPlumesMsg = particles()

    # Get raster parameters
    Robot_ID  = rospy.get_param("~RobotID")

    # Set node rate
    rate = rospy.Rate(50)

    # Create publishers
    consumedPlumesPublisher    = rospy.Publisher("consumedPlumes",    particles, queue_size=1)

    i = 0
    while not rospy.is_shutdown():



        if i > 200 and i < 400:
            Xfound = np.array([0.5], dtype=np.float32)
            Yfound = np.array([1.0], dtype=np.float32)
            Zfound = np.array([0.2], dtype=np.float32)
            Thetafound = np.array([0], dtype=np.float32)
            Qfound = np.array([2.58e-4], dtype=np.float32)
            Vfound = np.array([2], dtype=np.float32)
            Dyfound = np.array([3.8e-1], dtype=np.float32)
            Dzfound = np.array([3.0e-8], dtype=np.float32)
        elif i > 400:
            Xfound = np.array([0.5, 2], dtype=np.float32)
            Yfound = np.array([1.0, 2], dtype=np.float32)
            Zfound = np.array([0.2, 0.2], dtype=np.float32)
            Thetafound = np.array([0, 0], dtype=np.float32)
            Qfound = np.array([2.58e-4, 2.58e-4], dtype=np.float32)
            Vfound = np.array([2, 2], dtype=np.float32)
            Dyfound = np.array([3.8e-1, 3.8e-1], dtype=np.float32)
            Dzfound = np.array([3.0e-8, 3.0e-8], dtype=np.float32)
        else:
            Xfound = np.array([], dtype=np.float32)
            Yfound = np.array([], dtype=np.float32)
            Zfound = np.array([], dtype=np.float32)
            Thetafound = np.array([], dtype=np.float32)
            Qfound = np.array([], dtype=np.float32)
            Vfound = np.array([], dtype=np.float32)
            Dyfound = np.array([], dtype=np.float32)
            Dzfound = np.array([], dtype=np.float32)


        consumedPlumesMsg.X     = Xfound
        consumedPlumesMsg.Y     = Yfound
        consumedPlumesMsg.Z     = Zfound
        consumedPlumesMsg.theta = Thetafound
        consumedPlumesMsg.Q     = Qfound
        consumedPlumesMsg.v     = Vfound
        consumedPlumesMsg.Dy    = Dyfound
        consumedPlumesMsg.Dz    = Dzfound
        consumedPlumesMsg.weights = np.ones(len(Xfound))

        consumedPlumesPublisher.publish(consumedPlumesMsg)


        rate.sleep()
        i+=1




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
