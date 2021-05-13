#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def scan_callback(data):
    
    thres_shortest = 1.0 # units: m
    thres_obj_gap = 0.1  # units: m
    shortest = 0
    shortest_idx = 0
    shortest_flag = False

    for i in range(len(data.ranges)):
        if(data.ranges[i] == 0):
            pass
        elif(data.ranges[i] < thres_shortest):
            shortest = data.ranges[i]
            shortest_idx = i
            shortest_flag = True
    print(shortest_flag, " ", shortest_idx)

def listener():
    
    rospy.init_node('object_detector', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
