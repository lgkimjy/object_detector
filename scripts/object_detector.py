#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

thres_shortest = 2.0  # units: m
thres_obj_gap = 0.15  # units: m
alpha = 0.1
prev_theta = 0
obs_max_size = 20     # units : obstacle laser scan points

pub_obj = rospy.Publisher("/closest_object", LaserScan, queue_size=10)
pub_theta = rospy.Publisher("/closest_obj_theta", Float32, queue_size=10)
obs_msg = LaserScan()

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
    
    shortest = 0
    shortest_idx = 0
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
    obs_msg.header.frame_id = data.header.frame_id
    obs_msg.header.stamp = data.header.stamp
    obs_msg.angle_min = data.angle_min
    obs_msg.angle_max = data.angle_max
    obs_msg.angle_increment = data.angle_increment
    obs_msg.time_increment = data.time_increment
    obs_msg.scan_time = data.scan_time
    obs_msg.ranges = [0.0] * len(data.ranges)
    obs_msg.range_min = data.range_min
    obs_msg.range_max = data.range_max
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
        print("# of Laser Points : ", count, " : ", idx)
    pub_obj.publish(obs_msg)
    
    # obstacle angles, theta
    if(len(idx) == 0):
        theta = deg2rad(-90)
    else:
        theta = int( ((idx[0] + idx[-1]) / 2) * 86/len(data.ranges)- 149 + 90 + 16) # +90 = make forward theata to zero, +16=tilted degree of lidar
        #theta = deg2rad(theta)
    print("current theta : ", theta, ", lpf theta : " ,lpf(theta))
    prev_theta = lpf(theta)



if __name__ == '__main__':
    
    rospy.init_node('object_detector', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, LaserHandler)

    rospy.spin()
