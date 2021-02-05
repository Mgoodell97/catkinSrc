#!/usr/bin/env python

# Moving Average Filter

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
#from std_msgs.msg import Float32MultiArray
import csv
import numpy as np
import matplotlib.pyplot as plt

# for file creation
import os
import datetime


trace = [[],[],[],[], [], []]
# for timing convergence
gBestFit = 0
gBestPos = np.array([0,0])
#
# #### Creating file for test data
working_dir = "/home/darc/Dropbox/pso_data"
now = datetime.datetime.now()
test_file_dir = working_dir + "/" + str(now.year) + "-" + str(now.month) +'-' + str(now.day) + ' ' + str(now.hour) +'_' +str(now.minute) + '_' + str(now.second)
print test_file_dir
if not os.path.exists(test_file_dir):
    os.makedirs(test_file_dir)

file1 = open(test_file_dir + '/a1', "w")
file2 = open(test_file_dir + '/a2', "w")
file3 = open(test_file_dir + '/a3', "w")
####

class c_map:
    '''
        Class to hold a grid map for navigation. Reads in a map.txt file of the format
        0 - free cell, x - occupied cell, g - goal location, i - initial location.
        Additionally provides a simple transition model for grid maps and a convience function
        for displaying maps.
        '''

    def __init__(self, map_path=None):
        '''
        Constructor. Makes the necessary class variables. Optionally reads in a provided map
        file given by map_path.

        map_path (optional) - a string of the path to the file on disk
        '''
        self.origin = [0,0]
        self.rows = None
        self.cols = None
        self.goal = None
        self.init_pos = None
        self.con_grid = None
        if map_path is not None:
            self.read_map(map_path)

    def read_map(self, map_path):
        '''
        Read in a specified map file of the format described in the class doc string.

        map_path - a string of the path to the file on disk
        '''
        #g = np.loadtxt(open(map_path, "rb"), delimiter=",")
        reader = csv.reader(open(map_path, "rb"))
        x = list(reader)
        self.con_grid = np.array(x).astype("float")
        self.rows = self.con_grid.shape[0]
        self.cols = self.con_grid.shape[1]

    def get_con(self, position):
        index = [int(round(position[0] - self.origin[0])), int(round(position[1] - self.origin[1]))]
        if index[0] >= 0 and index[0] < self.rows and index[1] >= 0 and index[1] < self.cols:
            con = self.con_grid[index[0], index[1]]
            #print "index",
            #print index
        else:
            con = 0
        return con


def callback1(msg):
    global gBestPos, gBestFit
    r = rospy.Rate(1)
    pos = PoseStamped()
    pos.pose.position = msg.pose.position
    pos.header = msg.header
    frame = pos.header.frame_id
    pos.header.frame_id = frame[3]
    #print 'position ', [msg.pose.position.x, msg.pose.position.y]
    pos.pose.orientation.x = g.get_con([msg.pose.position.x, msg.pose.position.y])
    print frame[3]
    print 'position ', [msg.pose.position.x, msg.pose.position.y]
    print "con", pos.pose.orientation.x
    print "\n"

    # write to file
    if pos.pose.orientation.x > gBestFit:
        gBestFit = pos.pose.orientation.x
        gBestPos = [msg.pose.position.x, msg.pose.position.y]
    if int(frame[3]) == 1:
        file1.write(str(msg.pose.position.x) + ',' + str( msg.pose.position.y) + ',' + str(pos.pose.orientation.x) + ',' + str(gBestPos[0]) + ',' + str(gBestPos[1]) + '\n')
    elif int(frame[3]) == 2:
        file2.write(str(msg.pose.position.x) + ',' + str( msg.pose.position.y) + ',' + str(pos.pose.orientation.x) + ',' + str(gBestPos[0]) + ',' + str(gBestPos[1])+ '\n')
    elif int(frame[3]) == 3:
        file3.write(str(msg.pose.position.x) + ',' + str( msg.pose.position.y) + ',' + str(pos.pose.orientation.x) + ',' + str(gBestPos[0]) + ',' + str(gBestPos[1])+ '\n')

    # publish to mps_agent
    if int(frame[3]) == 1:
        r.sleep()
        MPS1.publish(pos)
        AgentMPS2.publish(pos)
        AgentMPS3.publish(pos)
        AgentMPS4.publish(pos)
    elif int(frame[3]) == 2:
        r.sleep()
        MPS2.publish(pos)
        AgentMPS1.publish(pos)
        AgentMPS3.publish(pos)
        AgentMPS4.publish(pos)
    elif int(frame[3]) == 3:
        r.sleep()
        MPS3.publish(pos)
        AgentMPS1.publish(pos)
        AgentMPS2.publish(pos)
        AgentMPS4.publish(pos)
    elif int(frame[3]) == 4:
        r.sleep()
        MPS4.publish(pos)
        AgentMPS1.publish(pos)
        AgentMPS2.publish(pos)
        AgentMPS3.publish(pos)

    if int(frame[3]) == 1:
        trace[0].append(msg.pose.position.x)
        trace[1].append(msg.pose.position.y)
        traj1.set_xdata(trace[0])
        traj1.set_ydata(trace[1])
    elif int(frame[3]) == 2:
        trace[2].append(msg.pose.position.x)
        trace[3].append(msg.pose.position.y)
        traj2.set_xdata(trace[2])
        traj2.set_ydata(trace[3])
    elif int(frame[3]) == 3:
        trace[4].append(msg.pose.position.x)
        trace[5].append(msg.pose.position.y)
        traj3.set_xdata(trace[4])
        traj3.set_ydata(trace[5])




    #print trace
def plot_wp_callback1(msg):
    wp1.set_xdata(msg.linear.x)
    wp1.set_ydata(msg.linear.y)
    print "wp 1"
def plot_wp_callback2(msg):
    wp2.set_xdata(msg.linear.x)
    wp2.set_ydata(msg.linear.y)
    print "wp 2", msg.linear
def plot_wp_callback3(msg):
    wp3.set_xdata(msg.linear.x)
    wp3.set_ydata(msg.linear.y)

def plot_wp_callback4(msg):
    wp4.set_xdata(msg.linear.x)
    wp4.set_ydata(msg.linear.y)

# plot best
def plotBest1(msg):
    pBest1.set_xdata(msg.pose.orientation.x)
    pBest1.set_ydata(msg.pose.orientation.y)
    #gBest.set_xdata(msg.pose.orientation.x)
    #gBest.set_ydata(msg.pose.orientation.y)
def plotBest2(msg):
    pBest2.set_xdata(msg.pose.orientation.x)
    pBest2.set_ydata(msg.pose.orientation.y)
def plotBest3(msg):
    pBest3.set_xdata(msg.pose.orientation.x)
    pBest3.set_ydata(msg.pose.orientation.y)
def plotBest4(msg):
    pBest4.set_xdata(msg.pose.orientation.x)
    pBest4.set_ydata(msg.pose.orientation.y)


if __name__ == '__main__':

    g = c_map('catkin_ws/src/obstacle_avoid/scripts/con_maps/testSite1.txt')
    plt.imshow(g.con_grid.transpose())
    plt.colorbar()
    plt.axis('tight')
    plt.show(block=False)
    # uav1 = Green
    # uav2 = Red
    # uav3 = Yellow
    wp1, = plt.plot(-1, -1, 'go')
    wp2, = plt.plot(80, 40, 'ro')
    wp3, = plt.plot(0,0, 'yo')
    traj1, = plt.plot(0, 0, 'g')
    traj2, = plt.plot(0, 0, 'r')
    traj3, = plt.plot(0,0, 'y')
    pBest1, = plt.plot(0,0,'gd', markersize=9)
    pBest2, = plt.plot(0,0,'rd', markersize=9)
    pBest3, = plt.plot(0,0,'yd', markersize=9)
    #gBest, = plt.plot(0,0,'y*', markersize=10)

    plt.pause(.05)
    AgentMPS1 = rospy.Publisher('uav1/agent_mps', PoseStamped, queue_size=1)
    AgentMPS2 = rospy.Publisher('uav2/agent_mps', PoseStamped, queue_size=1)
    AgentMPS3 = rospy.Publisher('uav3/agent_mps', PoseStamped, queue_size=1)
    AgentMPS4 = rospy.Publisher('uav4/agent_mps', PoseStamped, queue_size=1)
    MPS1 = rospy.Publisher('uav1/mps', PoseStamped, queue_size=1)
    MPS2 = rospy.Publisher('uav2/mps', PoseStamped, queue_size=1)
    MPS3 = rospy.Publisher('uav3/mps', PoseStamped, queue_size=1)
    MPS4 = rospy.Publisher('uav4/mps', PoseStamped, queue_size=1)
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("uav1/ground_truth_to_tf/pose", PoseStamped, callback1, queue_size=1)
    rospy.Subscriber("uav2/ground_truth_to_tf/pose", PoseStamped, callback1, queue_size=1)
    rospy.Subscriber("uav3/ground_truth_to_tf/pose", PoseStamped, callback1, queue_size=1)
    rospy.Subscriber("uav4/ground_truth_to_tf/pose", PoseStamped, callback1, queue_size=1)
    rospy.Subscriber("uav1/waypoint", Twist, plot_wp_callback1, queue_size=1)
    rospy.Subscriber("uav2/waypoint", Twist, plot_wp_callback2, queue_size=1)
    rospy.Subscriber("uav3/waypoint", Twist, plot_wp_callback3, queue_size=1)
    rospy.Subscriber("uav4/waypoint", Twist, plot_wp_callback4, queue_size=1)
    rospy.Subscriber("uav1/best", PoseStamped, plotBest1, queue_size=1)
    rospy.Subscriber("uav2/best", PoseStamped, plotBest2, queue_size=1)
    rospy.Subscriber("uav3/best", PoseStamped, plotBest3, queue_size=1)
    rospy.Subscriber("uav4/best", PoseStamped, plotBest4, queue_size=1)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        plt.draw()
        r.sleep()
        plt.pause(0.05)

    rospy.spin()
