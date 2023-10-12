#!/bin/bash

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch rslidar_sdk start.launch; \
exec bash"

# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore

sleep 5s  

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch pointcloud_to_laserscan sample_node.launch; \
exec bash"

sleep 5s 

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch ins550d gps.launch; \
exec bash"


echo “successfully started!”