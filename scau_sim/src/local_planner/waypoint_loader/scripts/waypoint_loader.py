#!/usr/bin/env python
import os
import csv
from geometry_msgs.msg import Quaternion, PoseStamped
from styx_msgs.msg import  Waypoint
from nav_msgs.msg import Path
from threading import Thread
import tf
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped

CSV_HEADER = ['x', 'y', 'yaw']


class WaypointLoader(object):
    def __init__(self):
        global  base_path
        global current_pose
        current_pose = PoseWithCovarianceStamped()
        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)

        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_path = rospy.Publisher('/scau/plan', Path, queue_size=1, latch=True)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback,queue_size=1)
        self.new_waypoint_loader(rospy.get_param('~path'))
        rospy.spin()

    def new_waypoint_loader(self, path):
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

    def load_waypoints(self, fname):
        base_path = Path()
        base_path.header.frame_id = 'map'
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
                goal.header.frame_id = 'map' 
                goal.pose.position.x = path_element.pose.position.x
                goal.pose.position.y =  path_element.pose.position.y
                goal.pose.orientation =  path_element.pose.orientation
                self.pub_goal.publish(goal)
                rospy.sleep(0.5)
                dist =1.5
                while dist > 1: 
                    dist = np.sqrt((current_pose.pose.pose.position.x - goal.pose.position.x)**2 + (current_pose.pose.pose.position.y - goal.pose.position.y)**2)


        t = Thread(target=publish_thread, args=(base_path,))
        t.start()
        self.pub_path.publish(base_path)
        return base_path


if __name__ == '__main__':
    try:
        WaypointLoader()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
