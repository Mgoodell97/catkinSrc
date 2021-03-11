#!/usr/bin/env python

import rospy #include <ros/ros.h> cpp equivalent
import numpy as np
from particle_filter.msg import estimatedGaussian



##################
# Functions
##################

# None

##################
# Global variables
##################

global UAV1_predict_gauss
global UAV2_predict_gauss
global UAV3_predict_gauss
UAV1_predict_gauss = estimatedGaussian()
UAV2_predict_gauss = estimatedGaussian()
UAV3_predict_gauss = estimatedGaussian()


##################
# Callbacks
##################

def UAV1_ep(UAV1_msg_ep):
    global UAV1_predict_gauss
    UAV1_predict_gauss= UAV1_msg_ep

def UAV2_ep(UAV2_msg_ep):
    global UAV2_predict_gauss
    UAV2_predict_gauss= UAV2_msg_ep

def UAV3_ep(UAV3_msg_ep):
    global UAV3_predict_gauss
    UAV3_predict_gauss= UAV3_msg_ep

##################
# Main function
##################

def main():
    rospy.init_node('hivemind')
    rate = rospy.Rate(10)

    try:
        plotUAV1 = rospy.get_param("UAV1")
    except:
        plotUAV1=False
    try:
        plotUAV2 = rospy.get_param("UAV2")
    except:
        plotUAV2 = False
    try:
        plotUAV3 = rospy.get_param("UAV3")
    except:
        plotUAV3 = False

    estimatedGaussian_pub = rospy.Publisher('hivemindEstimate', estimatedGaussian, queue_size=10)

    if plotUAV1:
        rospy.Subscriber("UAV1/gaussianEstimation", estimatedGaussian, UAV1_ep)

    if plotUAV2:
        rospy.Subscriber("UAV2/gaussianEstimation", estimatedGaussian, UAV2_ep)

    if plotUAV3:
        rospy.Subscriber("UAV3/gaussianEstimation", estimatedGaussian, UAV3_ep)

    averageGaussian = estimatedGaussian()

    while not rospy.is_shutdown():
        if plotUAV3:
            xPlume_estimated = (UAV1_predict_gauss.X+UAV2_predict_gauss.X+UAV3_predict_gauss.X)/3
            yPlume_estimated = (UAV1_predict_gauss.Y+UAV2_predict_gauss.Y+UAV3_predict_gauss.Y)/3
            zPlume_estimated = (UAV1_predict_gauss.Z+UAV2_predict_gauss.Z+UAV3_predict_gauss.Z)/3
            ThetaPlume_estimated =(UAV1_predict_gauss.Theta+UAV2_predict_gauss.Theta+UAV3_predict_gauss.Theta)/3
            QPlume_estimated = (UAV1_predict_gauss.Q+UAV2_predict_gauss.Q+UAV3_predict_gauss.Q)/3
            VPlume_estimated = (UAV1_predict_gauss.V+UAV2_predict_gauss.V+UAV3_predict_gauss.V)/3
            DyPlume_estimated = (UAV1_predict_gauss.Dy+UAV2_predict_gauss.Dy+UAV3_predict_gauss.Dy)/3
            DzPlume_estimated = (UAV1_predict_gauss.Dz+UAV2_predict_gauss.Dz+UAV3_predict_gauss.Dz)/3
        elif plotUAV2:
            xPlume_estimated = (UAV1_predict_gauss.X+UAV2_predict_gauss.X)/2
            yPlume_estimated = (UAV1_predict_gauss.Y+UAV2_predict_gauss.Y)/2
            zPlume_estimated = (UAV1_predict_gauss.Z+UAV2_predict_gauss.Z)/2
            ThetaPlume_estimated =(UAV1_predict_gauss.Theta+UAV2_predict_gauss.Theta)/2
            QPlume_estimated = (UAV1_predict_gauss.Q+UAV2_predict_gauss.Q)/2
            VPlume_estimated = (UAV1_predict_gauss.V+UAV2_predict_gauss.V)/2
            DyPlume_estimated = (UAV1_predict_gauss.Dy+UAV2_predict_gauss.Dy)/2
            DzPlume_estimated = (UAV1_predict_gauss.Dz+UAV2_predict_gauss.Dz)/2
        elif plotUAV1:
            xPlume_estimated = UAV1_predict_gauss.X
            yPlume_estimated = UAV1_predict_gauss.Y
            zPlume_estimated = UAV1_predict_gauss.Z
            ThetaPlume_estimated =UAV1_predict_gauss.Theta
            QPlume_estimated = UAV1_predict_gauss.Q
            VPlume_estimated = UAV1_predict_gauss.V
            DyPlume_estimated = UAV1_predict_gauss.Dy
            DzPlume_estimated = UAV1_predict_gauss.Dz

        averageGaussian.X     = xPlume_estimated
        averageGaussian.Y     = yPlume_estimated
        averageGaussian.Z     = zPlume_estimated
        averageGaussian.Theta = ThetaPlume_estimated
        averageGaussian.Q     = QPlume_estimated
        averageGaussian.V     = VPlume_estimated
        averageGaussian.Dy    = DyPlume_estimated
        averageGaussian.Dz    = DzPlume_estimated

        estimatedGaussian_pub.publish(averageGaussian)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
