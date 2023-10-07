#include "ros/ros.h"
#include "bits/stdc++.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "GeographicLib/LocalCartesian.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
//调库将经纬度转成东北天系下三维坐标
GeographicLib::LocalCartesian geo;
//设置初始位置经纬度,可通过代码获得
double latitude = 23.162676800000;
double longitude = 113.339565600000;
double altitude = 30.14500000000;
//声明三维坐标变量
double x, y, z;
//声明发布者
ros::Publisher pubOdom;
nav_msgs::Odometry msgOdom;
void callback(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg, const sensor_msgs::Imu::ConstPtr& imuMsg)
{
  static bool gpsInited = true;
  //只设置一次初始经纬高
  if (gpsInited)
  {
    geo.Reset(latitude, longitude, altitude);
    cout.precision(12);
    cout << fixed << gpsMsg->latitude << endl << gpsMsg->longitude << endl << gpsMsg->altitude << endl;
    gpsInited = false;
  }
  //经纬高转化为东北天坐标系下坐标
  geo.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, x, y, z);

  msgOdom.header.frame_id = "map";
  msgOdom.header.stamp = gpsMsg->header.stamp;
  msgOdom.pose.pose.position.x = x;
  msgOdom.pose.pose.position.y = y;
  msgOdom.pose.pose.position.z = z;
  //四元数由imu提供，认为该imu获得的四元数没有累积误差
  msgOdom.pose.pose.orientation = imuMsg->orientation;
  pubOdom.publish(msgOdom);
}

void velocityCallback(const geometry_msgs::Twist::ConstPtr& velocity_msg)
{
  // 接收到velocity话题的回调函数
  // 在这里处理接收到的坐标数据

  msgOdom.child_frame_id = "base_link";
  msgOdom.header.frame_id = "odom";
  msgOdom.header.stamp = ros::Time::now();
  msgOdom.twist.twist.linear.x = velocity_msg->linear.x;
  msgOdom.twist.twist.linear.y = velocity_msg->linear.y;
  msgOdom.twist.twist.linear.z = velocity_msg->linear.z;
  msgOdom.twist.twist.angular.x = velocity_msg->angular.x;
  msgOdom.twist.twist.angular.y = velocity_msg->angular.y;
  msgOdom.twist.twist.angular.z = velocity_msg->angular.z;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "odom_fusion");
  ros::NodeHandle n;
  pubOdom = n.advertise<nav_msgs::Odometry>("odom", 2000);
  //使用message_filters对gps和imu话题时间同步
  message_filters::Subscriber<sensor_msgs::NavSatFix> subGps(n, "/gps", 1000);
  message_filters::Subscriber<sensor_msgs::Imu> subImu(n, "/imu", 1000);
  ros::Subscriber velocity_sub = n.subscribe("/ASENSING_INS", 1000, velocityCallback);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subGps, subImu);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
}
