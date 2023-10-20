#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String  # 导入所需的消息类型
from geometry_msgs.msg import  Point

def publisher_node():
    rospy.init_node('end_point_pub', anonymous=True)  # 初始化ROS节点
    pub = rospy.Publisher('/planning/end_point', Point, queue_size=1)  # 创建一个话题发布者
    rate = rospy.Rate(8)  # 设置发布频率

    while not rospy.is_shutdown():
        goal = Point()  # 准备要发布的消息
        # goal.x = 82.0
        goal.x = 182.0
        goal.y = 0.40000000596
        goal.z = 0.0
        pub.publish(goal)  # 发布消息到话题
        rate.sleep()  # 控制发布频率

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
