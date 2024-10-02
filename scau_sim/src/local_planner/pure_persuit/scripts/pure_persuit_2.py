#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
from geometry_msgs.msg import Quaternion, PoseStamped, TwistStamped, Twist
from nav_msgs.msg import Path,Odometry
import tf
from tf import transformations
import rospy
import time

time.sleep(3)  			#休眠三秒后启动

# HORIZON = 1.0    不使用前瞻距离判断目标点
Velocity = 0.5 			#车辆行驶速度

class PurePersuit:
	def __init__(self):
		rospy.init_node('pure_persuit', log_level=rospy.DEBUG)
		rospy.Subscriber('/odom', Odometry, self.pose_cb, queue_size = 1)
		rospy.Subscriber('/scau/plan', Path, self.path_cb, queue_size = 1)

		self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

		self.currentpose = None
		self.currentpath = None
		self.target_reached = False    # 初始化是否到达目标点标志位

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

	def path_cb(self,plan_msg):
		# 从 Path 消息中提取poses数组坐标点
		path_points = plan_msg.poses

		# 迭代路径点
		for i in range(len(path_points)):
			#检查是否已经遍历到路径中的最后一个点
			if i <= len(path_points) - 1:
				# 检查是否已到达目标点
				if self.target_reached:
					# 移动到下一个路径点
					target_pose = path_points[i + 1]
					break
				else:
					target_pose = path_points[i]
					break

		# 设置要前往的路径点
		self.currentpath = target_pose
		rospy.logdebug('Already get Path~~~~')

	def calculateTwistCommand(self):
		
		# 获取目标位姿
		target_pose = self.currentpath

		targetSpeed = Velocity

		targetX = target_pose.pose.position.x
		targetY = target_pose.pose.position.y		
		currentX = self.currentpose.pose.position.x
		currentY = self.currentpose.pose.position.y
		# 获取车辆当前偏航角度（默认用弧度表示）
		quanternion = (self.currentpose.pose.orientation.x, self.currentpose.pose.orientation.y, self.currentpose.pose.orientation.z, self.currentpose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quanternion)	# 四元数转欧拉角
		yaw = euler[2]
		# 计算目标点与车辆当前朝向之间的相对角度以及距离
		# alpha = math.atan2(targetY - currentY, targetX - currentX) - yaw - 1.57079  #2024.5.1这里减去二分之π是因为当前运动方向是y轴正方向，需要考虑到相对转角(与实际测试场地有关)
		alpha = math.atan2(targetY - currentY, targetX - currentX) - yaw 
		l = math.sqrt(math.pow(currentX - targetX, 2) + math.pow(currentY - targetY, 2))
		if(l > 0.2):										#（可调）如果距离大于0.2（阈值），则通过纯跟踪算法计算新的航向角度，并生成相应的转向角度
			theta = math.atan(2 * 1.868 * math.sin(alpha) / l)
			# angle_degrees= -(math.degrees(theta)+90) + 90 									#2024.5.1这里加上90也是为了修正，但不清楚具体意义（调试而来）
			angle_degrees= -(math.degrees(theta)+90)
			if angle_degrees<90:
				turn=angle_degrees/90
				twistCmd = Twist() 
				rospy.loginfo('turn = {}'.format(turn))		#输出终端，用于调试
				if abs(turn) < 0.95:			# 防止过度转向
					twistCmd.linear.x=targetSpeed
					twistCmd.linear.z=0    		   # z值为0时控制主环闪黄灯，表示自动驾驶启动
					if abs(turn) < 0.15 :		#忽略细微的转向
						# rospy.loginfo('abs(turn)  < 0.95')
						twistCmd.angular.z = 0
					else:
						twistCmd.angular.z = -turn		#大于0为左转，小于0为右转，2024.5.2这里取负也是为了修正
		else:												# 如果距离 < 0.2，即快要到达时，设置为刹车或空
			twistCmd = Twist()
			twistCmd.linear.x=0			  # 控制刹车（测试时不行的话改成0）
			twistCmd.linear.z=1    				# 完成任务，亮蓝色(测试时也可以改为0)
			self.target_reached = True    # 标记目标点已到达

		print('linear:'+str(twistCmd.linear.x)+'  angular:'+str(twistCmd.angular.z))
		return twistCmd

if __name__ == '__main__':
	
    try:
        PurePersuit()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start PurePersuit node.')

