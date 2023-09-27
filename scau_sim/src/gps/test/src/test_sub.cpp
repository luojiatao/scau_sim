#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


ros::Publisher chatter_pub;

nav_msgs::Odometry Odom;
void velocityCallback(const geometry_msgs::Twist::ConstPtr& velocity_msg)
{
    // 接收到velocity话题的回调函数
    // 在这里处理接收到的坐标数据

    Odom.child_frame_id = "base_link";
    Odom.header.frame_id = "odom";
    Odom.header.stamp = ros::Time::now();
    Odom.twist.twist.linear.x=velocity_msg->linear.x;
    Odom.twist.twist.linear.y=velocity_msg->linear.y;
    Odom.twist.twist.linear.z=velocity_msg->linear.z;
    Odom.twist.twist.angular.x= velocity_msg->angular.x;
    Odom.twist.twist.angular.y= velocity_msg->angular.y;
    Odom.twist.twist.angular.z= velocity_msg->angular.z;



    
    chatter_pub.publish(Odom);

}

void axisCallback(const geometry_msgs::PoseStamped::ConstPtr& axis_msg)
{
    // 接收到/axis话题的回调函数
    // 在这里处理接收到的坐标数据
    Odom.pose.pose.position.x=axis_msg->pose.position.x;
    Odom.pose.pose.position.y=axis_msg->pose.position.y;
    Odom.pose.pose.position.z=axis_msg->pose.position.z;


    chatter_pub.publish(Odom);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coordinate_publisher");
    ros::NodeHandle nh;

    // 创建一个发布者，用于发布坐标数据到chatter_pub话题
    chatter_pub = nh.advertise<nav_msgs::Odometry>("/gps", 10);

    // 创建两个订阅者，分别用于接收velocity和/axis话题的数据
    ros::Subscriber velocity_sub = nh.subscribe("/ASENSING_INS", 10, velocityCallback);
    ros::Subscriber axis_sub = nh.subscribe("/odom", 10, axisCallback);


    ros::spin();

    return 0;
}