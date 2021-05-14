#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

thres_shortest = 0.5 # units: m
thres_obj_gap = 0.1  # units: m
obs_max_size = 5    # units : obstacle laser scan points

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
    obs_msg.header.frame_id = data.header.frame_id
    obs_msg.header.stamp = data.header.stamp
    obs_msg.angle_min = data.angle_min
    obs_msg.angle_max = data.angle_max
    obs_msg.angle_increment = data.angle_increment
    obs_msg.time_increment = data.time_increment
    obs_msg.scan_time = data.scan_time
    obs_msg.ranges = [0.0] * 270
    obs_msg.range_min = data.range_min
    obs_msg.range_max = data.range_max
    obs_msg.ranges[shortest_idx] = data.ranges[shortest_idx]
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

    pub = rospy.Publisher("/closest_obstacle", LaserScan, queue_size=10)

    rospy.Subscriber("/scan", LaserScan, LaserHandler)

    while not rospy.is_shutdown():
        pub.publish(obs_msg)
    rospy.spin()
