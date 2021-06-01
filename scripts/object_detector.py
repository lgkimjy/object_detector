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
alpha = 0.1
prev_theta = 0
obs_max_size = 20     # units : obstacle laser scan points

robot_id = rospy.get_param('robot_id', '')

pub_obj = rospy.Publisher("/"+ robot_id +"/object_detector/clustered_closest_obj", LaserScan, queue_size=10)
pub_theta = rospy.Publisher("/"+ robot_id +"/object_detector/clustered_closest_obj_theta", Float32, queue_size=10)
pub_point_debug = rospy.Publisher("/"+ robot_id +"/object_detector/closest_point_debug", PointStamped, queue_size=10)
pub_point = rospy.Publisher("/"+ robot_id +"/object_detector/obstacles", Obstacles, queue_size=10)

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

def LaserHandler(data):
    x = y = 0
    shortest = shortest_idx = 0
    shortest_flag = False
    idx = []
    print("# of Non Zero Laser Points : " ,dataCounter(data))   # -55~-125 : num of non zero laser points

    # FIND SHORTEST RANGES IDX
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
    print(shortest_flag, " : ", shortest_idx)

    # DATA COPY
    obs_msg = deepcopy(data)
    obs_msg.ranges = [0.0] * len(data.ranges)
    obs_msg.ranges[shortest_idx] = data.ranges[shortest_idx]
    obs_msg.ranges[shortest_idx] = shortest

    # CLUSTERING
    left_idx = right_idx = shortest_idx
    left_flag = right_flag = True
    l_tmp = r_tmp = shortest
    if(shortest_flag == True):
        count = 0
        for i in range(len(data.ranges)):

            if(left_idx >= len(data.ranges)-1):
                pass
            else:
                if(data.ranges[left_idx+1] == 0.0 and left_flag == True):
                    left_idx += 1
                elif(abs(l_tmp - data.ranges[left_idx + 1]) < thres_obj_gap and left_flag == True):
                    left_idx += 1
                    l_tmp = data.ranges[left_idx]
                    idx.append(left_idx)
                    count = count + 1
                    obs_msg.ranges[left_idx] = data.ranges[left_idx]
                else:
                    left_flag = False

            if(right_idx <= 0):
                pass
            else:
                if(data.ranges[right_idx - 1] == 0.0 and right_flag == True):
                    right_idx -= 1
                elif(abs(r_tmp - data.ranges[right_idx - 1]) < thres_obj_gap and right_flag == True):
                    right_idx -= 1
                    r_tmp = data.ranges[right_idx]
                    idx.append(right_idx)
                    count = count + 1
                    obs_msg.ranges[right_idx] = data.ranges[right_idx]
                else:
                    right_flag = False
            
            if(left_flag == False and right_flag == False):
                break

        idx.sort(reverse=True)
        # print("# of Laser Points : ", count, " : ", idx)
    pub_obj.publish(obs_msg)
    
    # obstacle angles, theta
    if(len(idx) == 0):
        theta = deg2rad(0)
        distance = 0
    else:
        # +90 = make forward degree default to zero, +16 = tilted degree of lidar
        theta = int( ((idx[0] + idx[-1]) / 2) * 86/len(data.ranges)- 149)
        distance = data.ranges[idx[int(count/2)]]
        x = distance * math.cos(deg2rad(theta))
        y = distance * math.sin(deg2rad(theta))
        #theta = deg2rad(theta)
    print("curr theta : ", theta, ", dist : ", distance)
    print("(x,y) = (", x, ",", y, ")")

    # DEBUG POINTS
    obs_point_debug_msg.header.frame_id = data.header.frame_id
    obs_point_debug_msg.point.x = distance * math.cos(deg2rad(theta-106))
    obs_point_debug_msg.point.y = distance * math.sin(deg2rad(theta-106))
    obs_point_debug_msg.point.z = 0
    pub_point_debug.publish(obs_point_debug_msg)
    
    obs_point_msg.header.frame_id = data.header.frame_id
    obs_point_msg.circles[0].center.x = x
    obs_point_msg.circles[0].center.y = y
    obs_point_msg.circles[0].center.z = 0
    pub_point.publish(obs_point_msg)


if __name__ == '__main__':
    
    rospy.init_node('object_detector_node', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, LaserHandler)

    rospy.spin()
