# 使用说明
### 1. 编译包，需要提前安装can_msgs消息包依赖
### 2. 运行程序，rosrun can_pkg serial_can_out.cpp
### 3. 控制面板1任务发送解锁cmd速度接收，使用turtlesim来进行模拟规划
### PS: mycan_pub.cpp是胡美琪的版本；serial_can_out是炜哥的，使用时将电脑接上can仪、运行该例程包、ros中发布cmd_vel即可