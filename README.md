![OIP](https://github.com/luojiatao/scau_sim/assets/108130094/e92792d6-d820-4f28-8013-978381957bb2)

# SCAU_SIM 工作空间

SCAU_SIM是华南农业大学用于FSAC赛道的仿真和算法平台，提供用于FSAC的一系列工具和功能。

## 启动步骤

1. 在工作空间下打开终端。
2. 在终端输入 `sh base.sh` 启动硬件驱动程序。
3. 在终端输入 `sh fsac_straight.sh` 启动直线加速仿真。
4. 在终端输入 `sh fsac_circle.sh` 启动八字环绕仿真。
5. 在终端输入`sh SR_fsac_straight.sh` 启动实车直线加速系统。

## 功能包介绍

### 1. gazebo_map

gmapping建图功能包。要使用此功能，需要先开启仿真功能和键盘控制功能，然后启动建图功能。运行 `rosrun map_server map_saver -f mapname` 在终端保存地图，地图默认保存在主目录下。

### 2. gazebo_nav

导航和定位功能包，包含move_base和amcl模块。导航所用的地图和配置文件存放在`map`子文件夹。

### 3. gazebo_pkg

仿真功能包。`world`子文件夹存放各赛道的world文件。

### 4. local_planner

包含路径加载模块和纯跟踪控制器模块。所有路径点的csv文件存放在`/local_planner/waypoint_loader/waypoints`下。纯跟踪脚本里pure_persuit_2.py用于不使用局部路径规划器的情况下进行控制。

### 5. teleop_twist_keyboard

键盘控制模块。运行python文件即可启动节点。

### 6. INS550D_ROS_Driver_V1.2

与ASENSING_INS_Driver_V1.02共同组成导航仪驱动功能包。用于发布初始imu和gps消息。

### 7. gps2odom

用于使用robot_localization融合IMU数据和GPS数据生成odom以定位。根据`gps2odom.yaml `文件配置参数。

### 8. rslidar_sdk

多线激光雷达驱动功能包（32线）。根据`config.yaml`文件配置参数。

### 9 . pointcloud_to_laserscan

多线激光雷达与单线激光雷达数据转换功能包。根据`sample_node.launch`文件配置参数。

### 10.robot_localization

定位功能包源码，此处仅用于了解学习参数配置，应用时请直接在gps2odom包中修改参数。
ekf和ukf是两种状态估计器的实现，使用了不同的算法，需要根据yaml配置文件配置参数，注意查看ekf_template.yaml。
详情参考ros官方文档：http://docs.ros.org/en/melodic/api/robot_localization/html/configuring_robot_localization.html
---

