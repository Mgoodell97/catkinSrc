#!/usr/bin/env python
# creates vector in direction of desired travel.


import rospy
import math
import MovingAverageFilter
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
import numpy as np

mode = rospy.get_param("mode")
bag = rospy.get_param("bag")
agent = rospy.get_param("AGENT_NUMBER")


if mode == 1:
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header
    import tf
    from collections import namedtuple
    import genpy
    import enif_iuc.msg
elif mode == 2:
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header
    import tf
    import genpy
elif mode == 3:
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header
    import tf
    import genpy
    import enif_iuc.msg
    from sensor_msgs.msg import Imu


sonar_dis = [] # arraty of sonar dist
seg = []
seg_mins = []
seg_size = 0
segment_scan = True
pub_full_scan = False
filter = MovingAverageFilter.MAFList(3)

yaw = 0
x_o = None
current_xy = [0,0]
agent_location = [[None,None]]*10
lat_lon = [0,0]

def norm_2pi(angle):
    while angle < 0:
        if angle < 0:
            angle = angle + 6.28
    while angle > 6.28:
        angle -= 6.28
    return angle

def read_sonar(msg):
    global sonar_dis
    sonar_dis = list(msg.ranges)

def process_laser(header):
    global hold_flag, sonar_dis, segment_scan
    ranges = list(header.ranges)
    angle_increment = header.angle_increment
    angle_min = header.angle_min
    # TODO would be more efficient to fill blind spot after segmenting
    ranges = fill_blind_spot(ranges, angle_min,header.angle_max, angle_increment)
    # if 1, does not consider laser
    #ranges = consider_sonar(ranges, angle_increment, sonar_dis, 0)
    if mode == 1:
        sonar_dis = get_sonar(ranges, angle_increment)
    if segment_scan:
        segment(ranges, angle_increment)
    #pub_new_laser(ranges, header)

def segment(ranges, angle_increment):
    global seg, seg_size, filter
    global current_xy, agent_location, yaw
    angle =  -3.14
    kk = 0
    seg = []
    min = 100
    seg_size = .2 # 10 deg
    angle_stop = -3.14 + seg_size
    while angle_stop < 3.14:
        while angle < (angle_stop):
            if ranges[kk] < min:
                min = ranges[kk]
            angle += angle_increment
            kk += 1
        seg += [min]
        angle_stop += seg_size
        min = 100
    global seg_mins # added b/c seg sometimes is not whole when change_speed() runs
    seg_mins = seg

    ######------- for avoiding other quads ----- ##############
    #print(agent_location)
    for agent in agent_location:
        if agent[0] is not None:
            vec =  np.array([agent[0] - current_xy[0], agent[1] - current_xy[1]]) # vector toward other agent
            agent_angle_g = math.atan2(vec[1], vec[0]) #  angle to other quad in global frame
            agent_angle = norm_2pi(agent_angle_g - yaw - math.pi)
            ii = int(agent_angle/seg_size)

            for kk in [-1,0,1]:
                jj = kk + ii
                if jj > len(seg_mins) -1:
                    jj -= len(seg_mins)
                r_real = np.linalg.norm(vec)
                r = r_real - 3
                if r < 1.2:
                    r = 1.2
                if r_real < seg_mins[jj]:
                    seg_mins[int(jj)] = r
    #####-------                          ------###############

    filter.add(seg_mins)
    pub_seg_laser(filter.ave(), seg_size)

    # if pub_full_scan:
    #     angle = -3.14
    #     kk = 1
    #     ranges_seg = []
    #     jj = 0
    #     steps_per_seg = int(math.ceil(seg_size / angle_increment))
    #     # print(seg_size/angle_increment)
    #     # print("steps per seg", steps_per_seg)
    #     while kk < len(ranges) and jj < (len(seg)):
    #
    #         for ii in range(0, steps_per_seg):
    #             ranges_seg += [seg[jj]]
    #             kk += 1
    #         # print("jj" , jj)
    #         jj += 1
    #     return ranges_seg

def get_sonar(ranges, angle_increment):
    # simulate sonar from laser scan data
    # not used when sim == 0
    angle = -3.14
    kk = 0
    ranges_sonar = []
    min = 100
    while angle < -1.832:
        angle += angle_increment
        kk += 1

    while angle < -1.309:
        if ranges[kk]  < min:
            min = ranges[kk]
        angle += angle_increment
        kk += 1
    ranges_sonar += [min]
    min = 100

    while angle < -0.262:
        angle += angle_increment
        kk += 1

    while angle < 0.262:
        if ranges[kk] < min:
            min = ranges[kk]
        angle += angle_increment
        kk += 1
    ranges_sonar += [min]
    min = 100

    while angle < 1.309:
        angle += angle_increment
        kk += 1

    while angle < 1.832:
        if ranges[kk] < min:
            min = ranges[kk]
        angle += angle_increment
        kk += 1
    ranges_sonar += [min]
    return ranges_sonar

def consider_sonar(ranges, angle_increment, sonar, only_sonar):
    cone = 0.34
    angle = -3.14
    kk = 0
    ranges_sonar = []
    if only_sonar == 1:
        print("ONLY SONAR")

    while angle < -(1.5708+cone/2): # start to gap
        if only_sonar == 1:
            ranges_sonar += [30]
        else:
            ranges_sonar += [ranges[kk]]
        angle += angle_increment
        kk += 1

    while angle < -(1.5708-cone/2): # sonar 1
        if only_sonar == 1:
            ranges_sonar += [sonar[0]]
        elif ranges[kk] < sonar[0] + .2:
            ranges_sonar += [ranges[kk]]
        else:
            ranges_sonar += [sonar[0]]
        angle += angle_increment
        kk += 1

    while angle < -cone/2: # gap
        if only_sonar == 1:
            ranges_sonar += [30]
        else:
            ranges_sonar += [ranges[kk]]
        angle += angle_increment
        kk += 1

    while angle < cone/2: # sonar 2
        if only_sonar == 1:
            ranges_sonar += [sonar[1]]
        elif ranges[kk] < sonar[1] + .2:
            ranges_sonar += [ranges[kk]]
        else:
            ranges_sonar += [sonar[1]]
        angle += angle_increment
        kk += 1

    while angle < (1.5708-cone/2): # gap
        if only_sonar == 1:
            ranges_sonar += [30]
        else:
            ranges_sonar += [ranges[kk]]
        angle += angle_increment
        kk += 1

    while angle < (1.5708+cone/2): # sonar 3
        if only_sonar == 1:
            ranges_sonar += [sonar[2]]
        elif ranges[kk] < sonar[2] + .2:
            ranges_sonar += [ranges[kk]]
        else:
            ranges_sonar += [sonar[2]]
        angle += angle_increment
        kk += 1

    while kk < len(ranges): # gap to stop
        if only_sonar == 1:
            ranges_sonar += [30]
        else:
            ranges_sonar += [ranges[kk]]
        angle += angle_increment
        kk += 1

    return ranges_sonar

def fill_blind_spot(ranges, angle_min, angle_max, angle_increment):
    #print("fillblind spot")
    global max_scan_dis
    length = len(ranges)
    jj = 0
    for M in ranges:  # replace inf and nan to 5
        if math.isnan(M) or math.isinf(M):
            ranges[jj] = 30
        if ranges[jj] > 30:
            ranges[jj] = 30
        if ranges[jj] < .5:
            ranges[jj] = 30.000
        jj += 1
    r2 = ranges[0]
    r1 = ranges[length - 1]
    angle = angle_min
    # get slope
    x1 = (math.cos(angle_max) * r1)
    y1 = (math.sin(angle_max) * r1)
    x2 = (math.cos(angle_min) * r2)
    y2 = (math.sin(angle_min) * r2)
    if (x2-x1 == 0):
        slope = 1000
    else :
        slope = (y2 - y1) / (x2 - x1)
    b = y2 - slope * x2
    while angle > -3.1415:
        gap_M = [b / (math.sin(angle) - slope * math.cos(angle))]
        ranges = gap_M + ranges
        angle -= angle_increment
    angle = angle_max
    while angle < 3.1415:
        gap_M = [b / (math.sin((angle)) - slope * math.cos(angle))]
        ranges = ranges + gap_M
        angle += angle_increment
    #kk = 0
    #for M in ranges: # changes laser scan distance
    #    if M > max_scan_dis:
    #        ranges[kk] = max_scan_dis
    #    kk += 1
    return ranges

def pub_seg_laser(valores, angle_increment):
    global mode
    idx = 0
    laser = LaserScan()
    header = Header()
    # creating header
    header.seq = idx
    ftime_now = rospy.get_time()
    header.stamp = genpy.rostime.Time(int(ftime_now) // 1, int((ftime_now % 1.0) * 1000000000))
    header.frame_id = "laser"

    laser.header = header
    laser.angle_min = 3.14
    laser.angle_max = -3.14
    laser.angle_increment = angle_increment
    laser.time_increment = 0
    laser.scan_time = 0
    laser.range_min = .1
    laser.range_max = 31 #header_scan.range_max
    laser.ranges = valores
    name = 'seg_scan'
    if mode == 1:
        name = name+'_sim'
    elif bag == 1:
        name = name + "_bag"
    pub = rospy.Publisher(name, LaserScan, queue_size=1)

    pub.publish(laser)
    idx+=1

def pub_new_laser(valores, header_scan):
    global sim
    idx = 0
    laser = LaserScan()
    header = Header()
    # creating header
    header.seq = idx
    ftime_now = rospy.get_time()
    header.stamp = genpy.rostime.Time(int(ftime_now) // 1, int((ftime_now % 1.0) * 1000000000))
    header.frame_id = "laser"

    laser.header = header
    laser.angle_min = 3.14
    laser.angle_max = -3.14
    laser.angle_increment = header_scan.angle_increment
    laser.time_increment = header_scan.time_increment
    laser.scan_time = header_scan.scan_time
    laser.range_min = header_scan.range_min
    laser.range_max = 31 #header_scan.range_max
    laser.ranges = valores
    name = 'new_scan'
    if mdoe == 1:
        name = name+'_sim'
    pub = rospy.Publisher(name, LaserScan,queue_size = 1)

    pub.publish(laser)
    idx+=1



def callback_avoid_agents(msg):
    global yaw, agent_location
    if mode == 1:
        # get agent number
        agent = int(msg.agent_number)
        # update agent location
        agent_location[agent] = [msg.mps.local_x, msg.mps.local_y]
    else:
        # get agent number
        agent = msg.agent_number
        # update agent location
        agent_location[agent] = latlon_2_m(msg.mps.GPS_latitude, msg.mps.GPS_longitude)


def set_yaw(msg): # gets pos and runs command function
    global yaw, current_xy
    if mode == 1:
        position = msg.pose.position
        quat = msg.pose.orientation
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        lat = position.x
        lon = position.y
        current_xy = [lat,lon]
    else:
        global pos_flag, lat_lon
        if mode == 2:
            quaternion = [msg.q3, msg.q1, msg.q2, msg.q0]  # switching the order
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = -euler[0]
        else: # Mavros
            quaternion = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = -euler[0] + math.pi / 2.0
            temp_xy = latlon_2_m(lat_lon[0], lat_lon[1])
            current_xy = [temp_xy[0], temp_xy[1]]
            #print('current', current_xy)


def set_lat_lon(msg):
    global lat_lon, x_o, pose_flag
    if mode == 1: # sim
        lat_lon = [msg.latitude, msg.longitude]
    else: # mavros and dji
        global lat_lon, x_o
        lat_lon = [msg.latitude, msg.longitude]
        if x_o is None:
            x_o = msg.latitude

def latlon_2_m(lat,lon):
    global x_o
    try:
        x_m = 6371000.0 * lat * math.pi / 180
        y_m = -6371000.0 * lon * math.pi / 180 * math.cos(x_o * math.pi / 180.0)

        return [x_m, y_m]
    except TypeError:

        print("No GPS. Not an issue if does not repeat")
        return [0,0]

if __name__ == '__main__':
    global mode
    rospy.init_node('listener', anonymous=True)
    if mode == 1:
        rospy.Subscriber("scan", LaserScan, process_laser, queue_size=1)
        rospy.Subscriber("sonars", LaserScan, read_sonar, queue_size = 1)
        #rospy.Subscriber('agent_mps', PoseStamped, callback_avoid_agents, queue_size=1)
        rospy.Subscriber("agent_mps_data", enif_iuc.msg.AgentMPS, callback_avoid_agents, queue_size=1)
        rospy.Subscriber("ground_truth_to_tf/pose", PoseStamped, set_yaw, queue_size=1)
        rospy.Subscriber("fix", NavSatFix, set_lat_lon, queue_size=1)
    elif mode == 2:
        rospy.Subscriber("scan", LaserScan, process_laser, queue_size=1)
        rospy.Subscriber("ultrasonic_scan", LaserScan, read_sonar,queue_size = 1)
    elif mode == 3:
        rospy.Subscriber("scan", LaserScan, process_laser, queue_size=1)
        rospy.Subscriber("ultrasonic_scan", LaserScan, read_sonar,queue_size = 1)
        rospy.Subscriber("agent_mps_data", enif_iuc.msg.AgentMPS, callback_avoid_agents, queue_size=1)
        rospy.Subscriber("mavros/global_position/global", NavSatFix,set_lat_lon, queue_size=1)
        rospy.Subscriber("mavros/imu/data", Imu, set_yaw, queue_size=1)


    rospy.spin()
