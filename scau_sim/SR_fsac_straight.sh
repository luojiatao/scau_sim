#!/bin/bash

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
sudo chmod 777 /dev/ttyUSB0; \
roslaunch ins550d gps.launch; \
exec bash"

sleep 5s

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch gps2odom gps2odom.launch; \
exec bash"

sleep 5s

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch rslidar_sdk start.launch; \
exec bash"

sleep 5s  

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch pointcloud_to_laserscan sample_node.launch; \
exec bash"

# 若需要实地建图，则还需要提前运行gmapping建图功能并保存所建地图

sleep 5s 

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch gazebo_nav amcl.launch; \
exec bash"

sleep 5s 

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch gazebo_nav racecar_nav_straight.launch; \
exec bash"

sleep 5s 

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch waypoint_loader waypoint_loader_straight.launch; \
exec bash"

sleep 5s 

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch pure_persuit pure_persuit.launch; \
exec bash"

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
rosrun can_pkg serial_can_out; \
exec bash"

echo “successfully started!”

