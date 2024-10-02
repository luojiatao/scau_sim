#!/usr/bin/env python
# -*- coding: utf-8 -*-

#此代码用于读取录制的odom包并计算瞬时速度的大小

import rospy
import rosbag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math

def calculate_distance(x1, y1, x2, y2):
    return y2 - y1
    # return math.sqrt(abs((x2 - x1)**2) +abs( (y2 - y1)**2))

def calculate_instantaneous_velocity():
    bag = rosbag.Bag('/home/wyc/2024-03-16-21-11-46.bag')  # Replace with your bag file path
    distance_threshold = 26.7  # Distance threshold in meters

    total_distance = 0
    previous_time = None
    previous_x = None
    previous_y = None
    print("Start reading messages...")
    for topic, msg, t in bag.read_messages(topics=['/odom']):
        if msg._type == 'nav_msgs/Odometry':
            # print("Received Odometry message.")
            # print("Message type:", type(msg))
            if msg.pose and msg.pose.pose and msg.pose.pose.position:
                # print("Processing Odometry message...")
                current_time = msg.header.stamp.to_sec()
                current_position = msg.pose.pose.position
                current_x = current_position.x
                current_y = current_position.y

            if previous_time is not None:
                distance = calculate_distance(previous_x, previous_y, current_x, current_y)
                total_distance += distance      #total_distance最后得到的是路程
                # print("Total distance so far: ", total_distance)
                time_interval = current_time - previous_time
                if time_interval != 0:
                    instantaneous_velocity = distance / time_interval
                    print("Instantaneous velocity at {} meters: {} m/s".format(total_distance, instantaneous_velocity))
                else:
                    print("Time interval is zero, unable to calculate instantaneous velocity.")

                if total_distance >= distance_threshold:
                    # print("Total distance exceeds threshold.")
                    # time_interval = current_time - previous_time
                    # instantaneous_velocity = distance / time_interval
                    # print("Instantaneous velocity at {} meters: {} m/s".format(distance_threshold, instantaneous_velocity))
                    break

            previous_time = current_time
            previous_x = current_x
            previous_y = current_y

    bag.close()
    print("Finished processing messages.")

if __name__ == "__main__":
    rospy.init_node('odom_velocity_calculator')
    print("Initializing node...")
    calculate_instantaneous_velocity()
