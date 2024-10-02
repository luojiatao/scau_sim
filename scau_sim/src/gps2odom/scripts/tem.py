#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf_conversions
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdomCorrector:
    def __init__(self):
        rospy.init_node('odom_corrector', anonymous=True)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.odom0_sub = rospy.Subscriber('/odom0', Odometry, self.odom_callback)

        self.saved_distance = None
        self.saved_angle = None
        self.prev_odom = None  # 保存上一次的odom0状态
        self.param_name_distance = '/odom0_distance'
        self.param_name_angle = '/odom0_angle'

        if rospy.has_param(self.param_name_distance) and rospy.has_param(self.param_name_angle):
            self.load_saved_state()
            rospy.loginfo("State loaded. Starting odom correction.")
        else:
            rospy.logwarn("No saved state found. Exiting...")
            rospy.signal_shutdown("No saved state to correct. Shutting down.")

    def load_saved_state(self):
        self.saved_distance = rospy.get_param(self.param_name_distance)
        self.saved_angle = rospy.get_param(self.param_name_angle)
        rospy.loginfo("Loaded saved state: distance=%f, angle=%f", self.saved_distance, self.saved_angle)

    def odom_callback(self, msg):
        # 如果odom0的状态发生变化才发布TF
        if self.has_odom_changed(msg):
            self.prev_odom = msg
            self.publish_transform(msg)

    def has_odom_changed(self, current_odom):
        """检查odom0状态是否发生变化"""
        if self.prev_odom is None:
            return True
        # 检查位置和方向是否有变化（可以根据阈值控制敏感度）
        prev_pos = self.prev_odom.pose.pose.position
        curr_pos = current_odom.pose.pose.position
        distance_moved = math.sqrt((curr_pos.x - prev_pos.x)**2 + (curr_pos.y - prev_pos.y)**2)
        return distance_moved > 0.001  # 阈值设置为1mm

    def publish_transform(self, msg):
        # 计算相对于原点的距离和角度
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        distance = math.sqrt(x**2 + y**2)  # 计算到原点的距离
        angle = math.atan2(y, x)  # 计算当前的角度

        # 根据给定逻辑修正角度
        if angle > 0:
            corrected_angle = angle - self.saved_angle
        else:
            corrected_angle = angle + self.saved_angle

        # 使用修正后的角度和距离计算新的x和y坐标
        corrected_x = distance * math.cos(corrected_angle)
        corrected_y = distance * math.sin(corrected_angle)

        # 发布修正后的TF
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'odom0'
        
        # 使用修正后的x和y
        transform.transform.translation.x = corrected_x
        transform.transform.translation.y = corrected_y
        transform.transform.translation.z = 0.0

        # 保持原始的四元数
        transform.transform.rotation = msg.pose.pose.orientation

        # 发布TF变换
        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    try:
        corrector = OdomCorrector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
