#!/usr/bin/env python

import rospy
import numpy as np
import math
import dji_sdk.msg

class lpf:
    def __init__(self, input=0, output=0, tc=0.4*math.pi, dt=0.02):
        self.lastIn  = input
        self.lastOut = output
        self.tc      = tc     # time constant
        self.dt      = dt     # sample time

    def update(self,curIn):
        self.lastOut = (self.tc*self.dt*curIn+self.tc*self.dt*self.lastIn-(self.tc*self.dt-2)*self.lastOut)/(2+self.tc*self.dt)
        self.lastIn  = curIn
        return self.lastOut
'''
def callback(msg):
    global pub, r_lpf, p_lpf, y_lpf, t_lpf 
    filtered_RC = dji_sdk.msg.RCChannels()
    filtered_RC.header = msg.header
        
    filtered_RC.roll  = r_lpf.update(msg.roll)
    filtered_RC.pitch = p_lpf.update(msg.pitch)
    filtered_RC.yaw = y_lpf.update(msg.yaw)
    filtered_RC.throttle = t_lpf.update(msg.throttle)
        
    pub.publish(filtered_RC)
        
if __name__ == "__main__":
    rospy.init_node('lpf', anonymous=True)
    rospy.Subscriber("dji_sdk/rc_channels", dji_sdk.msg.RCChannels, callback,queue_size=1 )
    pub = rospy.Publisher('filtered_rc', dji_sdk.msg.RCChannels,queue_size = 10)

    r = rospy.Rate(50)
    
    r_lpf = lpf()
    p_lpf = lpf()
    y_lpf = lpf()
    t_lpf = lpf()    

    while not rospy.is_shutdown():        
        r.sleep()

'''
