#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
from geometry_msgs.msg import PoseStamped
from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles
from quadnodes.msg import gaussian

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, pi, acos, sqrt, exp

from GaussianSensorPackage import computeGaussianPlumeMapNBNoClass, combinePlumesNew

import warnings

##################
# Functions
##################


##################
# Global variables
##################

xPlumesConsumed1     = np.array((), dtype=np.float32)  # [m]
yPlumesConsumed1     = np.array((), dtype=np.float32)  # [m]
zPlumesConsumed1     = np.array((), dtype=np.float32)  # [m]
thetaPlumesConsumed1 = np.array((), dtype=np.float32)  # [rads]
QsConsumed1          = np.array((), dtype=np.float32)  # [kg/s]
vsConsumed1          = np.array((), dtype=np.float32)  # [m/s]
DysConsumed1         = np.array((), dtype=np.float32)  # [m^2/s]
DzsConsumed1         = np.array((), dtype=np.float32)  # [m^2/s]

xPlumesConsumed2     = np.array((), dtype=np.float32)  # [m]
yPlumesConsumed2     = np.array((), dtype=np.float32)  # [m]
zPlumesConsumed2     = np.array((), dtype=np.float32)  # [m]
thetaPlumesConsumed2 = np.array((), dtype=np.float32)  # [rads]
QsConsumed2          = np.array((), dtype=np.float32)  # [kg/s]
vsConsumed2          = np.array((), dtype=np.float32)  # [m/s]
DysConsumed2         = np.array((), dtype=np.float32)  # [m^2/s]
DzsConsumed2         = np.array((), dtype=np.float32)  # [m^2/s]

xPlumesConsumed3     = np.array((), dtype=np.float32)  # [m]
yPlumesConsumed3     = np.array((), dtype=np.float32)  # [m]
zPlumesConsumed3     = np.array((), dtype=np.float32)  # [m]
thetaPlumesConsumed3 = np.array((), dtype=np.float32)  # [rads]
QsConsumed3          = np.array((), dtype=np.float32)  # [kg/s]
vsConsumed3          = np.array((), dtype=np.float32)  # [m/s]
DysConsumed3         = np.array((), dtype=np.float32)  # [m^2/s]
DzsConsumed3         = np.array((), dtype=np.float32)  # [m^2/s]

##################
# Callbacks
##################

def Robot1_consumed_gauss_cb(consumed_msg):
    global xPlumesConsumed1
    global yPlumesConsumed1
    global zPlumesConsumed1
    global thetaPlumesConsumed1
    global QsConsumed1
    global vsConsumed1
    global DysConsumed1
    global DzsConsumed1

    xPlumesConsumed1     = np.array(consumed_msg.X, dtype=np.float32)     # [m]
    yPlumesConsumed1     = np.array(consumed_msg.Y, dtype=np.float32)     # [m]
    zPlumesConsumed1     = np.array(consumed_msg.Z, dtype=np.float32)     # [m]
    thetaPlumesConsumed1 = np.array(consumed_msg.theta, dtype=np.float32) # [rads]
    QsConsumed1          = np.array(consumed_msg.Q, dtype=np.float32)     # [kg/s]
    vsConsumed1          = np.array(consumed_msg.v, dtype=np.float32)     # [m/s]
    DysConsumed1         = np.array(consumed_msg.Dy, dtype=np.float32)    # [m^2/s]
    DzsConsumed1         = np.array(consumed_msg.Dz, dtype=np.float32)    # [m^2/s]

def Robot2_consumed_gauss_cb(consumed_msg):
    global xPlumesConsumed2
    global yPlumesConsumed2
    global zPlumesConsumed2
    global thetaPlumesConsumed2
    global QsConsumed2
    global vsConsumed2
    global DysConsumed2
    global DzsConsumed2

    xPlumesConsumed2     = np.array(consumed_msg.X, dtype=np.float32)     # [m]
    yPlumesConsumed2     = np.array(consumed_msg.Y, dtype=np.float32)     # [m]
    zPlumesConsumed2     = np.array(consumed_msg.Z, dtype=np.float32)     # [m]
    thetaPlumesConsumed2 = np.array(consumed_msg.theta, dtype=np.float32) # [rads]
    QsConsumed2          = np.array(consumed_msg.Q, dtype=np.float32)     # [kg/s]
    vsConsumed2          = np.array(consumed_msg.v, dtype=np.float32)     # [m/s]
    DysConsumed2         = np.array(consumed_msg.Dy, dtype=np.float32)    # [m^2/s]
    DzsConsumed2         = np.array(consumed_msg.Dz, dtype=np.float32)    # [m^2/s]

def Robot3_consumed_gauss_cb(consumed_msg):
    global xPlumesConsumed3
    global yPlumesConsumed3
    global zPlumesConsumed3
    global thetaPlumesConsumed3
    global QsConsumed3
    global vsConsumed3
    global DysConsumed3
    global DzsConsumed3

    xPlumesConsumed3     = np.array(consumed_msg.X, dtype=np.float32)     # [m]
    yPlumesConsumed3     = np.array(consumed_msg.Y, dtype=np.float32)     # [m]
    zPlumesConsumed3     = np.array(consumed_msg.Z, dtype=np.float32)     # [m]
    thetaPlumesConsumed3 = np.array(consumed_msg.theta, dtype=np.float32) # [rads]
    QsConsumed3          = np.array(consumed_msg.Q, dtype=np.float32)     # [kg/s]
    vsConsumed3          = np.array(consumed_msg.v, dtype=np.float32)     # [m/s]
    DysConsumed3         = np.array(consumed_msg.Dy, dtype=np.float32)    # [m^2/s]
    DzsConsumed3         = np.array(consumed_msg.Dz, dtype=np.float32)    # [m^2/s]

##################
# Main function
##################

def main():
    rospy.init_node('plotGuassian')
    rate = rospy.Rate(30)

    particle_params = rospy.get_param("PF") # get params defined in launch file

    # Saves dictionary entries from launch file as variables
    for key,val in particle_params.items():
        exec(key + '=val')

    rospy.Subscriber("Robot_1/consumedPlumes",    particles, Robot1_consumed_gauss_cb)
    rospy.Subscriber("Robot_2/consumedPlumes",    particles, Robot2_consumed_gauss_cb)
    rospy.Subscriber("Robot_3/consumedPlumes",    particles, Robot3_consumed_gauss_cb)
    consumedPlumesPublisher    = rospy.Publisher("consumedPlumes",    particles, queue_size=1)

    consumedPlumesMsg = particles()

    Xfound     = np.array([], dtype=np.float32)
    Yfound     = np.array([], dtype=np.float32)
    Zfound     = np.array([], dtype=np.float32)
    Thetafound = np.array([], dtype=np.float32)
    Qfound     = np.array([], dtype=np.float32)
    Vfound     = np.array([], dtype=np.float32)
    Dyfound    = np.array([], dtype=np.float32)
    Dzfound    = np.array([], dtype=np.float32)

    while not rospy.is_shutdown():

        Xfound = np.append(xPlumesConsumed1, np.append(xPlumesConsumed2, xPlumesConsumed3))
        Yfound = np.append(yPlumesConsumed1, np.append(yPlumesConsumed2, yPlumesConsumed3))
        Zfound = np.append(zPlumesConsumed1, np.append(zPlumesConsumed2, zPlumesConsumed3))
        Thetafound = np.append(thetaPlumesConsumed1, np.append(thetaPlumesConsumed2, thetaPlumesConsumed3))
        Qfound = np.append(QsConsumed1, np.append(QsConsumed2, QsConsumed3))
        Vfound = np.append(vsConsumed1, np.append(vsConsumed2, vsConsumed3))
        Dyfound = np.append(DysConsumed1, np.append(DysConsumed2, DysConsumed3))
        Dzfound = np.append(DzsConsumed1, np.append(DzsConsumed2, DzsConsumed3))

        if len(Xfound) > 1: # merge plumes if needed
            plumeList = np.array([Xfound, Yfound, Zfound, Thetafound, Qfound, Vfound, Dyfound, Dzfound])

            Xfound, Yfound, Zfound, Thetafound, Qfound, Vfound, Dyfound, Dzfound = combinePlumesNew(plumeList, combineDistanceThreshold)

        consumedPlumesMsg.X     = Xfound
        consumedPlumesMsg.Y     = Yfound
        consumedPlumesMsg.Z     = Zfound
        consumedPlumesMsg.theta = Thetafound
        consumedPlumesMsg.Q     = Qfound
        consumedPlumesMsg.v     = Vfound
        consumedPlumesMsg.Dy    = Dyfound
        consumedPlumesMsg.Dz    = Dzfound

        consumedPlumesPublisher.publish(consumedPlumesMsg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
