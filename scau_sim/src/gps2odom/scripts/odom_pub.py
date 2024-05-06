#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3

def odom_publisher():
    # 初始化 ROS 节点
    rospy.init_node('odom_publisher', anonymous=True)
    
    # 创建一个发布者，发布类型为 nav_msgs/Odometry，话题名称为 /odom，队列大小为 10
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    
    # 设置 ROS 发布频率为 10Hz
    rate = rospy.Rate(10)  # 10Hz
    
    # 创建一个 nav_msgs/Odometry 消息对象
    odom_msg = Odometry()
    
    # 设置固定的数据
    # 位置信息
    odom_msg.pose.pose.position = Point(1.0, 0.0, 0.0)  
    
    # 姿态信息（四元数表示）
    odom_msg.pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # 四元数表示不旋转
    
    # 线速度信息
    odom_msg.twist.twist.linear = Vector3(0.0, 0.0, 0.0)  
    
    # 角速度信息
    odom_msg.twist.twist.angular = Vector3(0.0, 0.0, 0.0)  # no rotation
    
    while not rospy.is_shutdown():
        # 设置时间戳
        odom_msg.header.stamp = rospy.Time.now()
        
        # 发布消息
        odom_pub.publish(odom_msg)
        
        # 按照设定的频率休眠一段时间
        rate.sleep()

if __name__ == '__main__':
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint node.')
