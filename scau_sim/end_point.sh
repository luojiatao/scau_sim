#!/bin/bash

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch husky_navigation total_straight.launch; \
exec bash"

# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore

sleep 15s  

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch lidar_cluster lidar_cluster.launch; \
exec bash"

sleep 5s 

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch line_detector line_detector.launch; \
exec bash"

echo “successfully started!”

