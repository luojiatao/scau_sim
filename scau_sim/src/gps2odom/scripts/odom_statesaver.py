#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 此代码用于保存odom直线前进n米之后的位姿状态

import rospy
import math
from nav_msgs.msg import Odometry

class OdomStateSaver:
    def __init__(self):
        rospy.init_node('odom_state_saver', anonymous=True)

        self.odom0_sub = rospy.Subscriber('/odom0', Odometry, self.odom_callback)

        self.should_shutdown = False
        self.initial_position = None
        self.saved_distance = None
        self.saved_angle = None
        self.threshold = 1.0  # 前进X米的距离
        self.param_name_distance = '/odom0_distance'
        self.param_name_angle = '/odom0_angle'

    def calculate_distance_and_angle(self, x0, y0, x1, y1):
        # 计算欧几里得距离
        distance = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
        # 计算角度
        angle = math.atan2(y1 - y0, x1 - x0)
        return distance, angle

    def odom_callback(self, msg):
        position = msg.pose.pose.position

        # 第一次运行时，记录初始位置
        if self.initial_position is None:
            self.initial_position = position
            rospy.loginfo("Initial position recorded: x=%f, y=%f, z=%f" % (position.x, position.y, position.z))
            return

        # 计算从初始位置到当前点的距离和角度
        distance, angle = self.calculate_distance_and_angle(
            self.initial_position.x, self.initial_position.y, position.x, position.y)

        print("当前已前进{}米， 当前朝向角为{}".format(distance,angle))

        # 如果前进超过5米，保存距离和角度到ROS参数服务器
        if distance >= self.threshold and self.saved_distance is None and self.saved_angle is None:
            self.saved_distance = distance
            self.saved_angle = angle
            self.save_state_and_shutdown()

    def save_state_and_shutdown(self):
            #暂时不需要将其保存到ROS参数服务器，只需要终端打印出来看就行了    
            # rospy.set_param(self.param_name_distance, self.saved_distance)
            # rospy.set_param(self.param_name_angle, self.saved_angle)
            rospy.loginfo("Saved state to param server: distance=%f, angle=%f", self.saved_distance, self.saved_angle)
            self.should_shutdown = True  # 设置标志位，然后在主循环中检查

    def run(self):
        rospy.loginfo("启动odom_state_saver节点中...")
        # 创建一个频率为 10 Hz 的 ROS 速率对象，并且每调用一次 sleep()，程序就会暂停足够的时间以保持这个频率。这允许 ROS 在后台处理回调。
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown() and not self.should_shutdown:
            rate.sleep()  # Sleep to control loop rate

        # 当 should_shutdown 被设置时，关闭节点
        if self.should_shutdown:
            rospy.signal_shutdown("Reached 5 meters, state saved. Shutting down node...")

if __name__ == '__main__':
    try:
        state_saver = OdomStateSaver()
        state_saver.run()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
