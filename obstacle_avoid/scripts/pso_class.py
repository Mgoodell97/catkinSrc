#!/usr/bin/env python
import numpy as np
import random
import rospy
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import Header
import genpy

bag = rospy.get_param("bag")

class pso:
    # filter for angles with magnitudes
    def __init__(self, box, position, stay_time, vel, use_wp=True, percent_random=0):
        self.box = box  # [(bottom left x, y), height, length]
        # Personal (Cognitive)
        self.pBestFit = 0
        self.pBestPos = np.array([0,0])
        # Global (Social)
        self.gBestFit = 0
        self.gBestPos = np.array([0,0])
        print "box", box
        self.waypoint = np.array([self.box[0][0] + np.random.rand()*self.box[1], self.box[0][1] + np.random.rand()*self.box[2]])
        print "init wp", self.waypoint
        self.position = np.array([position[0], position[1]])
        self.time_out = np.linalg.norm(self.waypoint - self.position) * 1.2
        self.loop_count = 0
        self.use_wp = use_wp
        self.percent_random = percent_random
        self.stay_time = stay_time
        self.vel = vel

    def update_self(self, position, fit):
        self.position = np.array([position[0], position[1]])
        if fit > self.pBestFit:
            self.pBestFit = fit
            self.pBestPos = self.position
        if fit > self.gBestFit:
            self.gBestFit = fit
            self.gBestPos = self.position

    def update_agents(self, agent_pos, agent_fit):
        # check if new reading is best globally
        if agent_fit > self.gBestFit:
            self.gBestFit = agent_fit
            self.gBestPos = agent_pos
        # check if personal is best globally
       # print "pfit", self.pBestFit, 'gfit', self.gBestFit
        if self.pBestFit > self.gBestFit:
            self.gBestFit = self.pBestFit
            self.gBestPos = self.pBestPos

    def get_waypoint(self):
        # TODO if stuck in while redue may waypoint closest position in box

        c1 = 1
        c2 = 2
        redue = True
        reached = False
        print 'dis',np.linalg.norm(self.waypoint - self.position)
        print "time out", self.time_out
        if not self.use_wp or (np.linalg.norm(self.waypoint - self.position) < 1 or self.time_out < 1):  # if not using wps, or wp reached, or time out
            if np.linalg.norm(self.waypoint - self.position) < 1:
                reached = True
            self.loop_count = 0
            while (redue or rospy.is_shutdown()) and self.loop_count < 100:  # if wp is not in box and ros not shutdown
                R1 = np.random.rand()  # random numbers
                R2 = np.random.rand()
                rand_vel = (np.random.rand(2) - .5) * 2
                #print 'rand_vel', rand_vel

                if 1: #random.random() < 1 - self.percent_random:  # using PSO
                    if self.gBestFit == 0: # if not gBest complete random #print "no best yet"
                        temp = [(random.random() - 0.5) * 2 * self.box[1], (random.random() - .5) * 2 * self.box[2]]
                    elif self.pBestFit == 0 or self.pBestFit == self.gBestFit: # if not pBest yet do not consider in PSO
                        #w = .5 + random.random() / 2
                        #print "w", w
                        temp = rand_vel * 10 + c2 * (self.gBestPos - self.position) * R2
                    else: # pBest and gBest
                        #w = .5 + random.random() / 2
                        #print "w", w
                        temp = rand_vel * 10 + c1 * (self.pBestPos - self.position) * R1 + c2 * (self.gBestPos - self.position) * R2
                else:
                    temp = [(random.random() - 0.5) * 2 * self.box[1], (random.random() - .5) * 2 * self.box[2]]
                #print 'toss', temp
                self.waypoint = self.position +temp
                # Check if wp in box
                if self.waypoint[0] > self.box[0][0] + self.box[1] or self.waypoint[0] < self.box[0][0] or self.waypoint[1] < self.box[0][1] + self.box[2] or self.waypoint[1] > self.box[0][1]: # works
                    redue = True
                else: # set new wp
                    redue = False
                    self.time_out = np.linalg.norm(self.waypoint - self.position) * 1.2 + self.stay_time
                    return self.waypoint, reached
                self.loop_count += 1

            if self.loop_count >= 100:
                # find closet corner
                print "wont get in the box, set to corner"
                self.loop_count = 0

                corners = [self.box[0], (self.box[0][0]+self.box[1], self.box[0][1]), (self.box[0][0], self.box[0][1] + self.box[2]), (self.box[0][0] + self.box[1], self.box[0][1] + self.box[2])]
                min_dis = float('Inf')
                for c in corners:
                    dis = np.linalg.norm(self.position - c)
                    if dis < min_dis:
                        self.waypoint = c
                        min_dis = dis
                    self.time_out = (np.linalg.norm(self.waypoint - self.position) * 1.2)/self.vel + self.stay_time

        self.time_out -= 1
        return self.waypoint, reached

    def pub_con(self):
        idx = 0
        pos = PoseStamped()
        # creating header
        header = Header()
        header.seq = idx
        ftime_now = rospy.get_time()
        header.stamp = genpy.rostime.Time(int(ftime_now) // 1, int((ftime_now % 1.0) * 1000000000))
        header.frame_id = "laser"
        pos.header = header
        # global position (x,y) and con (z)
        pos.pose.position.x = self.gBestPos[0]
        pos.pose.position.y = self.gBestPos[1]
        pos.pose.position.z = self.gBestFit

        # personal best
        pos.pose.orientation.x = self.pBestPos[0]
        pos.pose.orientation.y = self.pBestPos[1]
        pos.pose.orientation.z = self.pBestFit
        if bag == 0:
            pub = rospy.Publisher('best', PoseStamped, queue_size=1)
        else:
            pub = rospy.Publisher('best_bag', PoseStamped, queue_size=1)
        pub.publish(pos)
        idx += 1
        return [self.pBestPos[0], self.pBestPos[1]], self.pBestFit, [self.gBestPos[0], self.gBestPos[1]], self.gBestFit
