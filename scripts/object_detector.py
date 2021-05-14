#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

thres_shortest = 1.0 # units: m
thres_obj_gap = 0.2  # units: m
obs_max_size = 50    # units : obstacle laser scan points

obs_msg = LaserScan()

def LaserHandler(data):
    
    shortest = 0
    shortest_idx = 0
    shortest_flag = False

    # FIND SHORTEST RANGES IDX
    for i in range(len(data.ranges)):
        if(data.ranges[i] == 0):
            pass
        elif(data.ranges[i] < thres_shortest):
            shortest = data.ranges[i]
            shortest_idx = i
            shortest_flag = True
    # print(shortest_flag, " ", shortest_idx)

    # CLUSTERING
    left_idx = right_idx = shortest_idx
    left_flag = right_flag = True
    obs_msg.ranges[shortest_idx] = data.ranges[shortest_idx]
    obs_msg.header.frame_id = "laser_frame"
    for i in range(obs_max_size):

        if(data.ranges[left_idx] - data.ranges[left_idx + i] < thres_obj_gap and left_flag == True):
            left_idx = left_idx + i
            obs_msg.ranges[left_idx] = data.ranges[left_idx]
        else:
            left_flag = False

        if(data.ranges[right_idx] - data.ranges[right_idx - i] < thres_obj_gap and right_flag == True):
            right_idx = right_idx + i
            obs_msg.ranges[right_idx] = data.ranges[right_idx]
        else:
            right_flag = False

if __name__ == '__main__':
    
    rospy.init_node('object_detector', anonymous=True)

    pub = rospy.Publisher("/closest_obstacle", LaserScan, anonymous=True)

    rospy.Subscriber("/scan", LaserScan, LaserHandler)

    while not rospy.is_shutdown():
        pub.publish(obs_msg)
    rospy.spin()