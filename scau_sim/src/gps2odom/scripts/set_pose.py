#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def set_pose():
    rospy.init_node('set_pose_publisher', anonymous=True)
    pub = rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size=10)
    rate = rospy.Rate(30)  

    while not rospy.is_shutdown():
        # 创建一个 PoseWithCovarianceStamped 消息
        pose_msg = PoseWithCovarianceStamped()

        # 填充消息的位置信息
        pose_msg.pose.pose.position.x = -0.1
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0

        # 填充消息的姿态信息
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0

        # 发布消息
        pub.publish(pose_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        set_pose()
    except rospy.ROSInterruptException:
        pass
