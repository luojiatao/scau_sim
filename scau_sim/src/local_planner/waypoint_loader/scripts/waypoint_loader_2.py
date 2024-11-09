#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 此代码用于不使用雷达不使用amcl的方案下直接订阅odom作为定位并使用/scau/plan话题发布csv文件预载入点

import os
import csv
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path,Odometry
from threading import Thread
import tf
import rospy
import numpy as np


CSV_HEADER = ['x', 'y', 'yaw']


class WaypointLoader(object):       
    def __init__(self):
        global  base_path
        global current_pose
        current_pose = Odometry()
        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)

        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)    
        self.pub_path = rospy.Publisher('/scau/plan', Path, queue_size=1, latch=True)                            #path表示整个规划路径
        rospy.Subscriber("/odom", Odometry, self.pose_callback,queue_size=1)
        self.new_waypoint_loader(rospy.get_param('~path'))
        rospy.spin()

    def new_waypoint_loader(self, path):    # 用于加载路径点
        if os.path.isfile(path):
            self.load_waypoints(path)
            rospy.loginfo('Waypoint Loded')
        else:
            rospy.logerr('%s is not a file', path)


    def quaternion_from_yaw(self, yaw): 
        return Quaternion(x=0.0, y=0.0, z=np.sin(yaw/2.0), w=np.cos(yaw/2.0))

    def pose_callback(self,msg):
        global current_pose
        current_pose = msg

    def load_waypoints(self, fname):        # 从CSV文件中加载路径点
        base_path = Path()
        base_path.header.frame_id = 'odom'
        with open(fname) as wfile:
            reader = csv.DictReader(wfile, CSV_HEADER)
            for wp in reader:
                path_element = PoseStamped() 
                path_element.pose.position.x =  float(wp['x'])
                path_element.pose.position.y = float(wp['y'])
                q = self.quaternion_from_yaw(float(wp['yaw']))
                path_element.pose.orientation = q 
                base_path.poses.append(path_element) 

        def publish_thread(base_path):      
            for path_element in base_path.poses:
                goal = PoseStamped()
                goal.header.frame_id = 'odom' 
                goal.pose.position.x = path_element.pose.position.x
                goal.pose.position.y =  path_element.pose.position.y
                goal.pose.orientation =  path_element.pose.orientation
                self.pub_goal.publish(goal)
                rospy.sleep(0.5)                            #将线程执行暂停 0.5 秒，允许机器人有时间开始朝目标移动。
                dist =1.5
                while dist > 1:             # 机器人到目标点的距离大于1时，继续等待。（即只有当距离小于等于1后才发布新的目标点给路径规划）
                    dist = np.sqrt((current_pose.pose.pose.position.x - goal.pose.position.x)**2 + (current_pose.pose.pose.position.y - goal.pose.position.y)**2)


        t = Thread(target=publish_thread, args=(base_path,))   # 在单独线程中发布机器人的目标点，并等待机器人到达目标点。
        t.start()
        self.pub_path.publish(base_path)    # 主线程发布全局路径信息以供可视化或进一步处理
        return base_path


if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
