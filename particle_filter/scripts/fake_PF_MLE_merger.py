#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
from enif_iuc.msg import AgentEstimatedGaussian

global quadID
global quad_agent_msg_dict
global quad_agent_msg_dict_previous
quad_agent_msg_dict = {}
quad_agent_msg_dict_previous = {}

def agent_mle_gauss_cb(agent_mle_gauss_msg_tmp):
    quad_agent_msg_dict_previous = quad_agent_msg_dict.copy()

    quad_agent_msg_dict[agent_mle_gauss_msg_tmp.agent_number] = agent_mle_gauss_msg_tmp.estimatedgaussian

    # Update PF with new particle and renormalize the weights
    if agent_mle_gauss_msg_tmp.agent_number not in quad_agent_msg_dict_previous.keys():
        # Add particle here
        print "Added agent's", agent_mle_gauss_msg_tmp.agent_number, "particle"
    elif quad_agent_msg_dict[agent_mle_gauss_msg_tmp.agent_number] != quad_agent_msg_dict_previous[agent_mle_gauss_msg_tmp.agent_number]:
        # Add particle here
        print "Added agent's", agent_mle_gauss_msg_tmp.agent_number, "particle"

    # else do not update partile filter
    # just ignore repeat data

def main():
    rospy.init_node('Fake_PF_MLE_merger') #node name is offb_node

    global quadID
    quadID = rospy.get_param("quadID")               #  [-]

    rate = rospy.Rate(10)

    if quadID == 1:
        rospy.Subscriber("/agent_mle_gauss_data1", AgentEstimatedGaussian, agent_mle_gauss_cb)

    if quadID == 2:
        rospy.Subscriber("/agent_mle_gauss_data2", AgentEstimatedGaussian, agent_mle_gauss_cb)

    if quadID == 3:
        rospy.Subscriber("/agent_mle_gauss_data3", AgentEstimatedGaussian, agent_mle_gauss_cb)

    # Let ros take over
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
