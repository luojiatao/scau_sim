#!/bin/bash

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch husky_navigation total_straight.launch; \
exec bash"

# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore

sleep 15s  

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch waypoint_loader waypoint_loader_straight.launch; \
exec bash"

sleep 5s 

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch pure_persuit pure_persuit.launch; \
exec bash"

echo “successfully started!”

