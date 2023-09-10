# scau_sim
FSAC仿真平台

#启动步骤：
1.在工作空间下打开终端。
2.在终端输入 sh fsac.sh 启动所有功能。(即~/scau_sim$ sh fsac.sh )

#功能包介绍：
1.gazebo_map：gmapping建图功能包，需要提前开启仿真功能和键盘控制功能，再启动建图功能，终端运行 rosrun map_server map_saver -f mapname保存地图，地图默认保存于主目录下。
2.gazebo_nav：导航和定位功能包，包含move_base和amcl模块，导航所用的地图和配置文件存放在该map子文件夹。
3.gazebo_pkg：仿真功能包，world子文件夹存放各赛道的world文件。
4.local_planner：包含路径加载模块和纯跟踪控制器模块，/local_planner/waypoint_loader/waypoints下存放当前赛道所有路径点的csv文件。
5.teleop_twist_keyboard：键盘控制模块，运行python文件即启动节点。
