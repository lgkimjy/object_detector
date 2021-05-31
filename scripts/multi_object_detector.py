#!/usr/bin/env python3
import rospy
import math
from copy import deepcopy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from obstacle_detector.msg import Obstacles, CircleObstacle

thres_shortest = 2.0  # units: m
thres_obj_gap = 0.15  # units: m
obs_max_size = 30     # units : obstacle laser scan points
obs_min_size = 15
alpha = 0.1
prev_theta = 0

pub_obj = rospy.Publisher("/object_detector/clustered_closest_obj", LaserScan, queue_size=10)
pub_theta = rospy.Publisher("/object_detector/clustered_closest_obj_theta", Float32, queue_size=10)
pub_point_debug = rospy.Publisher("/object_detector/closest_point_debug", PointStamped, queue_size=10)
pub_point = rospy.Publisher("/object_detector/obstacles", Obstacles, queue_size=10)

filtered_msg = LaserScan()
obs_msg = LaserScan()
obs_point_debug_msg = PointStamped()
obs_point_msg = Obstacles()

tmp = CircleObstacle()
obs_point_msg.circles.append(tmp)

def deg2rad(data):
    return math.radians(data)

def lpf(data):
    return alpha * prev_theta + (1-alpha) * data

def dataCounter(data):
    j=0
    for i in range(len(data.ranges)):
        if(data.ranges[i] != 0.0):
            j += 1
    return j

def findShortest(data):
    shortest = shortest_idx = 0
    shortest_flag = False
    flag = True
    for i in range(len(data.ranges)):
        if(data.ranges[i] == 0):
            pass
        elif(data.ranges[i] < thres_shortest and flag == True):
            shortest = data.ranges[i]
            shortest_idx = i
            shortest_flag = True
            flag = False
        elif(data.ranges[i] < shortest and flag == False):
            shortest = data.ranges[i]
            shortest_idx = i

    return shortest_flag, shortest, shortest_idx

def LaserHandler(data):
    x = y = 0
    idx = []
    # print("# of Non Zero Laser Points : " ,dataCounter(data))   # -63~-149 : num of non zero laser points

    ### DATA COPY
    filtered_msg = deepcopy(data)
    filtered_msg.ranges = list(filtered_msg.ranges) 
    obs_msg = deepcopy(data)
    obs_msg.ranges = [0.0] * len(data.ranges)
    
    while(1):
        ### FIND SHORTEST RANGES IDX
        shortest_flag, shortest, shortest_idx = findShortest(filtered_msg)
        print(shortest_flag, " : ", shortest_idx)

        ### CLUSTERING
        left_idx = right_idx = shortest_idx
        left_flag = right_flag = True
        l_tmp = r_tmp = shortest
        if(shortest_flag == True):
            idx.append(shortest_idx)
            for i in range(len(filtered_msg.ranges)):
                if(left_idx >= len(filtered_msg.ranges)-1):
                    left_flag = False
                    pass
                else:
                    if(filtered_msg.ranges[left_idx+1] == 0.0 and left_flag == True):
                        left_idx += 1
                    elif(abs(l_tmp - filtered_msg.ranges[left_idx + 1]) < thres_obj_gap and left_flag == True):
                        left_idx += 1
                        l_tmp = filtered_msg.ranges[left_idx]
                        idx.append(left_idx)
                        obs_msg.ranges[left_idx] = filtered_msg.ranges[left_idx]
                        filtered_msg.ranges[left_idx] = 0.0
                    else:
                        left_flag = False

                if(right_idx <= 0):
                    right_flag = False
                    pass
                else:
                    if(filtered_msg.ranges[right_idx - 1] == 0.0 and right_flag == True):
                        right_idx -= 1
                    elif(abs(r_tmp - filtered_msg.ranges[right_idx - 1]) < thres_obj_gap and right_flag == True):
                        right_idx -= 1
                        r_tmp = filtered_msg.ranges[right_idx]
                        idx.append(right_idx)
                        obs_msg.ranges[right_idx] = filtered_msg.ranges[right_idx]
                        filtered_msg.ranges[right_idx] = 0.0
                    else:
                        right_flag = False
                
                if(left_flag == False and right_flag == False):
                    filtered_msg.ranges[shortest_idx] = 0.0
                    idx.sort(reverse=True)
                    print("# of Laser Points : ", len(idx), " : ", idx)
                    break
                
            if(len(idx) < obs_min_size):
                for i in range(len(idx)):
                    obs_msg.ranges[idx[i]] = 0.0
            idx = []
            left_flag = right_flag = True

        else:
            # print("breaks")
            break

    pub_obj.publish(obs_msg)
    
    # # obstacle angles, theta
    # if(len(idx) == 0):
    #     theta = deg2rad(0)
    #     distance = 0
    # else:
    #     # +90 = make forward degree default to zero, +16 = tilted degree of lidar
    #     theta = int( ((idx[0] + idx[-1]) / 2) * 86/len(data.ranges)- 149 + 90 + 16)
    #     distance = data.ranges[idx[int(count/2)]]
    #     x = distance * math.cos(deg2rad(theta))
    #     y = distance * math.sin(deg2rad(theta))
    #     #theta = deg2rad(theta)
    # print("curr theta : ", theta, ", dist : ", distance)
    # print("(x,y) = (", x, ",", y, ")")

    # ### DEBUG POINTS
    # obs_point_debug_msg.header.frame_id = data.header.frame_id
    # obs_point_debug_msg.point.x = distance * math.cos(deg2rad(theta-106))
    # obs_point_debug_msg.point.y = distance * math.sin(deg2rad(theta-106))
    # obs_point_debug_msg.point.z = 0
    # pub_point_debug.publish(obs_point_debug_msg)
    
    # obs_point_msg.header.frame_id = data.header.frame_id
    # obs_point_msg.circles[0].center.x = x
    # obs_point_msg.circles[0].center.y = y
    # obs_point_msg.circles[0].center.z = 0
    # pub_point.publish(obs_point_msg)


if __name__ == '__main__':
    
    rospy.init_node('object_detector_node', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, LaserHandler)

    rospy.spin()