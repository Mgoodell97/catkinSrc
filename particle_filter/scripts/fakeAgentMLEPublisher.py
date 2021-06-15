#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
from enif_iuc.msg import AgentEstimatedGaussian

import numpy as np

def randomNumAtoB(a,b):
    return ((b - a) * np.random.random_sample() + a)

def main():
    rospy.init_node('Fake_agent_MLE_publisher')

    quadID = rospy.get_param("quadID")               #  [-]

    rate = rospy.Rate(1)

    if quadID == 1:
        mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)

    if quadID == 2:
        mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data3', AgentEstimatedGaussian, queue_size=1)

    if quadID == 3:
        mle_gauss_pub1 = rospy.Publisher('/agent_mle_gauss_data1', AgentEstimatedGaussian, queue_size=1)
        mle_gauss_pub2 = rospy.Publisher('/agent_mle_gauss_data2', AgentEstimatedGaussian, queue_size=1)

    fake_MLE_Msg = AgentEstimatedGaussian()

    count = 0

    while not rospy.is_shutdown(): # while roscore is running do this. if not, stop... Executes Function
        # Publish random data
        if count < 5:
            fake_MLE_Msg.agent_number            = quadID
            fake_MLE_Msg.estimatedgaussian.X     = 1
            fake_MLE_Msg.estimatedgaussian.Y     = 1
            fake_MLE_Msg.estimatedgaussian.Z     = 1
            fake_MLE_Msg.estimatedgaussian.Theta = 1
            fake_MLE_Msg.estimatedgaussian.Q     = 1
            fake_MLE_Msg.estimatedgaussian.V     = 1
            fake_MLE_Msg.estimatedgaussian.Dy    = 1
            fake_MLE_Msg.estimatedgaussian.Dz    = 1
            fake_MLE_Msg.estimatedgaussian.W     = 1

        elif (count > 10 and count < 15):
            fake_MLE_Msg.agent_number            = quadID
            fake_MLE_Msg.estimatedgaussian.X     = 1
            fake_MLE_Msg.estimatedgaussian.Y     = 1
            fake_MLE_Msg.estimatedgaussian.Z     = 1
            fake_MLE_Msg.estimatedgaussian.Theta = 1
            fake_MLE_Msg.estimatedgaussian.Q     = 1
            fake_MLE_Msg.estimatedgaussian.V     = 1
            fake_MLE_Msg.estimatedgaussian.Dy    = 1
            fake_MLE_Msg.estimatedgaussian.Dz    = 1
            fake_MLE_Msg.estimatedgaussian.W     = 1
        else:
            fake_MLE_Msg.agent_number            = quadID
            fake_MLE_Msg.estimatedgaussian.X     = randomNumAtoB(0,20)
            fake_MLE_Msg.estimatedgaussian.Y     = randomNumAtoB(0,20)
            fake_MLE_Msg.estimatedgaussian.Z     = randomNumAtoB(0,20)
            fake_MLE_Msg.estimatedgaussian.Theta = randomNumAtoB(0,20)
            fake_MLE_Msg.estimatedgaussian.Q     = randomNumAtoB(0,20)
            fake_MLE_Msg.estimatedgaussian.V     = randomNumAtoB(0,20)
            fake_MLE_Msg.estimatedgaussian.Dy    = randomNumAtoB(0,20)
            fake_MLE_Msg.estimatedgaussian.Dz    = randomNumAtoB(0,20)
            fake_MLE_Msg.estimatedgaussian.W     = randomNumAtoB(0,20)

        mle_gauss_pub1.publish(fake_MLE_Msg)
        mle_gauss_pub2.publish(fake_MLE_Msg)

        count+=1

        rate.sleep() # have you met the rospy.Rate? if not wait some amount of time.

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
