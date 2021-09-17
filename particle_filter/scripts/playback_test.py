#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
#import numpy
from Particle_Filter_Utils import *
import matplotlib.pyplot as plt
import random
# from sensor_msgs.msg import Joy

from quadnodes.msg import gaussian
from olfaction_msgs.msg import gas_sensor
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from mps_driver.msg import MPS

from particle_filter.msg import estimatedGaussian
from particle_filter.msg import particles

from enif_iuc.msg import AgentEstimatedGaussian

from mavros_msgs.msg import State #include <mavros_msgs/State.h>

global chem_reading
global current_state

global robotSensor

plumeMsg = estimatedGaussian()

robotSensorReadingPoseMPS_R1 = MPS() # chem reading in gauss env
robotSensorReadingPoseMPS_R2 = MPS() # chem reading in gauss env
robotSensorReadingPoseMPS_R3 = MPS() # chem reading in gauss env
robotSensorReadingPoseMPS_R4 = MPS() # chem reading in gauss env
robotSensorReadingPoseMPS_R5 = MPS() # chem reading in gauss env

def MPSSensorAndPose_cb_R1(MPSMsg):
    global robotSensorReadingPoseMPS
    global sensorMsg_cb_flag
    robotSensorReadingPoseMPS_R1 = MPSMsg
    sensorMsg_cb_flag = True

def MPSSensorAndPose_cb_R2(MPSMsg):
    global robotSensorReadingPoseMPS
    global sensorMsg_cb_flag
    robotSensorReadingPoseMPS_R2 = MPSMsg
    sensorMsg_cb_flag = True

def MPSSensorAndPose_cb_R3(MPSMsg):
    global robotSensorReadingPoseMPS
    global sensorMsg_cb_flag
    robotSensorReadingPoseMPS_R3 = MPSMsg
    sensorMsg_cb_flag = True

def MPSSensorAndPose_cb_R4(MPSMsg):
    global robotSensorReadingPoseMPS
    global sensorMsg_cb_flag
    robotSensorReadingPoseMPS_R4 = MPSMsg
    sensorMsg_cb_flag = True

def MPSSensorAndPose_cb_R5(MPSMsg):
    global robotSensorReadingPoseMPS
    global sensorMsg_cb_flag
    robotSensorReadingPoseMPS_R5 = MPSMsg
    sensorMsg_cb_flag = True

def main():
    rospy.init_node('PlaybackTest') # name your node
      ## Initialization of Particles and Sensor ##
    rate = rospy.Rate(5)

    rospy.Subscriber("/Robot_1/mps_data", MPS, MPSSensorAndPose_cb_R1)
    rospy.Subscriber("/Robot_2/mps_data", MPS, MPSSensorAndPose_cb_R2)
    rospy.Subscriber("/Robot_3/mps_data", MPS, MPSSensorAndPose_cb_R3)
    rospy.Subscriber("/Robot_4/mps_data", MPS, MPSSensorAndPose_cb_R4)
    rospy.Subscriber("/Robot_5/mps_data", MPS, MPSSensorAndPose_cb_R5)

    print("Robot:", robotSensor," data: ", chem_reading, "RobotPose:", robotPoseNew)
    rate.sleep() # have you met the rospy.Rate? if not wait some amount of time.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
