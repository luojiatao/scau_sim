#!/usr/bin/env python

import math

from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped, Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Path


import tf
from tf import transformations
import rospy

HORIZON = 1.0
Velocity = 1.0

class PurePersuit:
	def __init__(self):
		rospy.init_node('pure_persuit', log_level=rospy.DEBUG)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_cb, queue_size = 1)
		rospy.Subscriber('/planning/ref_path', Path, self.path_cb, queue_size = 1)

		self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

		self.currentpose = None
		self.currentpath = None

		self.loop()

	def loop(self):
		rate = rospy.Rate(20)
		rospy.logwarn("pure persuit starts")
		while not rospy.is_shutdown():
			if self.currentpose  and self.currentpath:
				twistCommand = self.calculateTwistCommand()
				self.twist_pub.publish(twistCommand)
			rate.sleep()

	def pose_cb(self,data):
		self.currentpose = data.pose

	def path_cb(self,data):
		self.currentpath = data

	def calculateTwistCommand(self):
		lad = 0.0 #look ahead distance accumulator
		targetIndex = len(self.currentpath.poses) - 1
		for i in range(len(self.currentpath.poses)):
			if((i+1) < len(self.currentpath.poses)):
				this_x = self.currentpath.poses[i].pose.position.x
				this_y = self.currentpath.poses[i].pose.position.y
				next_x = self.currentpath.poses[i+1].pose.position.x
				next_y = self.currentpath.poses[i+1].pose.position.y
				lad = lad + math.hypot(next_x - this_x, next_y - this_y)
				if(lad > HORIZON):
					targetIndex = i+1
					break


		targetpose = self.currentpath.poses[targetIndex]

		targetSpeed = Velocity

		targetX = targetpose.pose.position.x
		targetY = targetpose.pose.position.y		
		currentX = self.currentpose.pose.position.x
		currentY = self.currentpose.pose.position.y
		#get vehicle yaw angle
		quanternion = (self.currentpose.pose.orientation.x, self.currentpose.pose.orientation.y, self.currentpose.pose.orientation.z, self.currentpose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quanternion)
		yaw = euler[2]
		#get angle difference
		alpha = math.atan2(targetY - currentY, targetX - currentX) - yaw
		l = math.sqrt(math.pow(currentX - targetX, 2) + math.pow(currentY - targetY, 2))
		if(l > 0.5):
			theta = math.atan(2 * 1.868 * math.sin(alpha) / l)
			# #get twist command
			twistCmd = Twist()
			twistCmd.linear.x = targetSpeed
			twistCmd.angular.z = theta 
		else:
			twistCmd = Twist()
			twistCmd.linear.x = 0
			twistCmd.angular.z = 0

		return twistCmd


if __name__ == '__main__':
    try:
        PurePersuit()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start motion control node.')

