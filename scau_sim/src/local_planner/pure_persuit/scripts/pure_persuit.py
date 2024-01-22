#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped, Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Path
import tf
from tf import transformations
import rospy
import time

time.sleep(3)  			#休眠三秒后启动

HORIZON = 1.0
Velocity = 0.5

class PurePersuit:
	def __init__(self):
		rospy.init_node('pure_persuit', log_level=rospy.DEBUG)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_cb, queue_size = 1)
		rospy.Subscriber('/local_path', Path, self.path_cb, queue_size = 1)

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
		lad = 0.0 						#  前瞻距离累加器
		targetIndex = len(self.currentpath.poses) - 1	#初始化目标点索引为路径上最后一个点的索引，假设最初目标为路径的最末端。
		#  遍历路径上的所有路径点
		for i in range(len(self.currentpath.poses)):
			if((i+1) < len(self.currentpath.poses)):
				# 获取当前路径点和下一个路径点的坐标信息
				this_x = self.currentpath.poses[i].pose.position.x
				this_y = self.currentpath.poses[i].pose.position.y
				next_x = self.currentpath.poses[i+1].pose.position.x
				next_y = self.currentpath.poses[i+1].pose.position.y
				# 计算当前路径点与下一个路径点之间的直线距离，并将其累加到前瞻距离累加器中。
				lad = lad + math.hypot(next_x - this_x, next_y - this_y)
				# 判断是否达到前瞻距离阈值
				if(lad > HORIZON):
					# 获取目标点索引
					targetIndex = i+1
					break

		# 获取目标位姿
		targetpose = self.currentpath.poses[targetIndex]

		targetSpeed = Velocity

		targetX = targetpose.pose.position.x
		targetY = targetpose.pose.position.y		
		currentX = self.currentpose.pose.position.x
		currentY = self.currentpose.pose.position.y
		# 获取车辆当前偏航角度（默认用弧度表示）
		quanternion = (self.currentpose.pose.orientation.x, self.currentpose.pose.orientation.y, self.currentpose.pose.orientation.z, self.currentpose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quanternion)	# 四元数转欧拉角
		yaw = euler[2]
		# 计算目标点与车辆当前朝向之间的相对角度以及距离
		alpha = math.atan2(targetY - currentY, targetX - currentX) - yaw
		l = math.sqrt(math.pow(currentX - targetX, 2) + math.pow(currentY - targetY, 2))
		if(l > 0.5):										#如果距离大于0.5（阈值），则通过纯跟踪算法计算新的航向角度，并生成相应的转向角度
			theta = math.atan(2 * 1.868 * math.sin(alpha) / l)
			angle_degrees= -(math.degrees(theta)+90)
			#设置异常停止
			if angle_degrees<90:
				turn=angle_degrees/90
				twistCmd = Twist() 
				if abs(turn) < 0.95:			# 防止过度转向
					twistCmd.linear.x=targetSpeed
					twistCmd.linear.z=0    		   # z的线速度为0用于控制主环闪黄灯，表示自动驾驶启动
					if abs(turn) < 0.15 :
						twistCmd.angular.z = 0
					else:
						twistCmd.angular.z = turn
		else:												# 如果距离 < 0.5，即快要到达全局路径终点
			twistCmd = Twist()
			twistCmd.linear.x=-0.9			  # 控制刹车（测试时不行的话改成0）
			twistCmd.linear.z=1    				#完成任务，亮蓝色

		print('linear:'+str(twistCmd.linear.x)+'  angular:'+str(twistCmd.angular.z))
		return twistCmd

if __name__ == '__main__':
	
    try:
        PurePersuit()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start PurePersuit node.')

