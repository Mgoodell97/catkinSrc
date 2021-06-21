#!/usr/bin/env python
import rospy #include <ros/ros.h> cpp equivalent
from enif_iuc.msg import AgentEstimatedGaussian

# global quadID
# global quad_agent_msg_dict
# global quad_agent_msg_dict_previous
# global quad_agent_msg_flag
quad_agent_msg_dict = {}
quad_agent_msg_dict_previous = {}
quad_agent_msg_flag = {}

def agent_mle_gauss_cb(agent_mle_gauss_msg_tmp):
    quad_agent_msg_dict_previous = quad_agent_msg_dict.copy()

    quad_agent_msg_dict[agent_mle_gauss_msg_tmp.agent_number] = agent_mle_gauss_msg_tmp.estimatedgaussian

    # Update PF with new particle and renormalize the weights
    if agent_mle_gauss_msg_tmp.agent_number not in quad_agent_msg_dict_previous.keys():
        quad_agent_msg_flag[agent_mle_gauss_msg_tmp.agent_number] = True # Raise flag to add particle in next loop

    elif quad_agent_msg_dict[agent_mle_gauss_msg_tmp.agent_number] != quad_agent_msg_dict_previous[agent_mle_gauss_msg_tmp.agent_number]:
        quad_agent_msg_flag[agent_mle_gauss_msg_tmp.agent_number] = True # Raise flag to add particle in next loop

    # else do not update partile filter
    # just ignore repeat data

def main():
    rospy.init_node('Fake_PF_MLE_merger') #node name is offb_node

    quadID = rospy.get_param("quadID")               #  [-]

    rate = rospy.Rate(1)

    if quadID == 1:
        rospy.Subscriber("/agent_mle_gauss_data1", AgentEstimatedGaussian, agent_mle_gauss_cb)

    if quadID == 2:
        rospy.Subscriber("/agent_mle_gauss_data2", AgentEstimatedGaussian, agent_mle_gauss_cb)

    if quadID == 3:
        rospy.Subscriber("/agent_mle_gauss_data3", AgentEstimatedGaussian, agent_mle_gauss_cb)

    # Let ros take over
    while not rospy.is_shutdown():

        for key, value in quad_agent_msg_flag.items():
            if value:
                print "Added agent's", key, "particle"
                print (quad_agent_msg_dict[key])
                quad_agent_msg_flag[key] = False

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
