#!/usr/bin/env python
import math
from geometry_msgs.msg import  Twist,PoseWithCovarianceStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Path
import tf
from tf import transformations
import rospy
import nav_msgs.msg as nav_msgs
HORIZON = 1.0
L = 0.5
T = 0.5
Velocity = 1.0

class PurePersuit:
	def __init__(self):
		rospy.init_node('pure_persuit', log_level=rospy.DEBUG)

		# rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_cb, queue_size = 1)
		rospy.Subscriber('/odometry/filtered', nav_msgs.Odometry, self.pose_cb, queue_size=1)  

		rospy.Subscriber('/local_path', Path, self.path_cb, queue_size = 1)
		# rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.path_cb, queue_size = 1)

		# self.left_vel_pub =rospy.Publisher('/rear_left_velocity_controller/command', Float64, queue_size = 10)
		# self.right_vel_pub = rospy.Publisher('/rear_right_velocity_controller/command', Float64, queue_size = 10)
		# self.left_steer_pub = rospy.Publisher('/left_bridge_position_controller/command', Float64, queue_size = 10)
		# self.right_steer_pub = rospy.Publisher('/right_bridge_position_controller/command', Float64, queue_size = 10)
		self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
		self.currentpose = None
		self.currentpath = None

		self.loop()

	def loop(self):
		rate = rospy.Rate(20)
		rospy.logwarn("pure persuit starts")
		while not rospy.is_shutdown():
			if self.currentpose  and self.currentpath:
				self.calculateTwistCommand()
			rate.sleep()

	# def pose_cb(self,data):
	# 	self.currentpose = data.pose

	def pose_cb(self, msg):
		self.currentpose = msg.pose 


	def path_cb(self,data):
		self.currentpath = data
		
	def ackermann_steering_control(self, velocity, radian):
		global left_angle, left_speed, right_angle, right_speed
		# print ('radian = '), radian	
		if radian > 0:
			inside_radius = L / math.tan(radian) - T / 2
			outside_radius = L / math.tan(radian) + T / 2
		else:
			outside_radius = L / math.tan(radian) - T / 2
			inside_radius = L / math.tan(radian) + T / 2
	
		outside_speed = velocity * ( 1 + T * math.tan(abs(radian)) / ( 2 * L ) )
		inside_speed = velocity * ( 1 - T * math.tan(abs(radian)) / ( 2 * L ) )
	
		inside_angle = math.atan( L / inside_radius )
		outside_angle = math.atan( L / outside_radius )
	
		if radian > 0:
			left_angle = outside_angle
			left_speed = outside_speed
			right_angle = inside_angle
			right_speed = inside_speed
		else:
			right_angle = outside_angle
			right_speed = outside_speed
			left_angle = inside_angle
			left_speed = inside_speed
		
		# print('left_speed = ', left_speed, 'right_speed = ', right_speed)

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
			theta = math.atan(2 * L * math.sin(alpha) / l)
			self.ackermann_steering_control(targetSpeed, -theta)
			# #get twist command
			left_steer = Float64()
			right_steer = Float64()
			left_vel = Float64()
			right_vel = Float64()
			left_steer.data = left_angle
			right_steer.data = right_angle
			left_vel.data = left_speed
			right_vel.data = right_speed 
		else:
			left_steer = Float64()
			right_steer = Float64()
			left_vel = Float64()
			right_vel = Float64()
			left_steer.data = 0
			right_steer.data = 0
			left_vel.data = 0
			right_vel.data = 0
			
		# self.left_vel_pub.publish(left_vel)
		# self.right_vel_pub.publish(right_vel)
		# self.left_steer_pub.publish(left_steer)
		# self.right_steer_pub.publish(right_steer)
		cmd_msg=Twist()
		cmd_msg.linear.x=(left_vel.data + right_vel.data)/2
		cmd_msg.angular.z=-(left_steer.data + right_steer.data)/2
		self.twist_pub.publish(cmd_msg)

if __name__ == '__main__':
    try:
        PurePersuit()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start motion control node.')

