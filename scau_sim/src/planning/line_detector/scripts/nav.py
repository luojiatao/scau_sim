#!/usr/bin/env python
import os
import csv
from geometry_msgs.msg import  PoseStamped
from nav_msgs.msg import Path
from threading import Thread
import tf
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped


class WaypointLoader(object):
    def __init__(self):

        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.Subscriber("/planning/end_point", PoseStamped, self.pose_callback,queue_size=1)
        self.new_waypoint_loader()
        rospy.spin()


    def pose_callback(self,msg):
        global goal_pose
        goal_pose = msg

    def new_waypoint_loader(self, path):
        goal = PoseStamped()
        goal.header.frame_id = 'map' 
        goal.pose.position.x = goal_pose.pose.pose.position.x
        goal.pose.position.y =  goal_pose.pose.pose.position.y
        # goal.pose.orientation =  goal_pose.pose.pose.position.
        self.pub_goal.publish(goal)

if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
