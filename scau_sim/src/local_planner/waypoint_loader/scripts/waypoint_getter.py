#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped, Quaternion
import tf
import math

class PointCloudProcessor:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('pointcloud_to_pose', anonymous=True)

        # 订阅PointCloud类型的话题
        self.pointcloud_sub = rospy.Subscriber('/perception/lidar_cluster', PointCloud, self.pointcloud_callback)

        # 创建PoseStamped类型的话题发布者
        self.pose_pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)

        # 设置x和y的阈值范围
        self.x_threshold = rospy.get_param('~x_threshold', 0.1)
        self.y_threshold = rospy.get_param('~y_threshold', 0.1)

    def pointcloud_callback(self, msg):
        points = msg.points
        paired_points = []

        # 配对点并筛选符合条件的点对
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                dx = abs(points[i].x - points[j].x)
                dy = abs(points[i].y - points[j].y)
                if  dx <= self.x_threshold and self.y_threshold <= dy <= self.y_threshold:      #记得改
                    paired_points.append((points[i], points[j]))

        # 计算每对点的中点和中垂线朝向
        for point1, point2 in paired_points:
            mid_x = (point1.x + point2.x) / 2.0
            mid_y = (point1.y + point2.y) / 2.0
            mid_z = (point1.z + point2.z) / 2.0

            # 计算中垂线的朝向(跑八字环绕时还需要根据实际情况添加一些逻辑判断修正朝向)
            angle = math.atan2(point2.y - point1.y, point2.x - point1.x) + math.pi / 2
            if 90 <= angle <= 270:         #这里暂时只考虑直线赛道情况下的修正
                angle = angle - 180
            quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)

            # 创建PoseStamped消息
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = msg.header.frame_id
            pose.pose.position.x = mid_x
            pose.pose.position.y = mid_y
            pose.pose.position.z = mid_z
            pose.pose.orientation = Quaternion(*quaternion)

            # 发布PoseStamped消息
            self.pose_pub.publish(pose)

if __name__ == '__main__':
    try:
        processor = PointCloudProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
