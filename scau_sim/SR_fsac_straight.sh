#!/bin/bash

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
sudo chmod 777 /dev/ttyUSB0; \
roslaunch ins550d gps.launch; \
exec bash"

# 两个roslauch之间需要间隔一段时间，否则会相互抢占roscore
sleep 5s

gnome-terminal --tab -- bash -c "\
source devel/setup.bash; \
roslaunch gps2odom gps2odom.launch; \
exec bash"

sleep 5s

# gnome-terminal --tab -- bash -c "\
# source devel/setup.bash; \
# roslaunch rslidar_sdk start.launch; \
# exec bash"

# sleep 5s  

# gnome-terminal --tab -- bash -c "\
# source devel/setup.bash; \
# roslaunch pointcloud_to_laserscan sample_node.launch; \
# exec bash"

# sleep 5s 

# gnome-terminal --tab -- bash -c "\
# source devel/setup.bash; \
# roslaunch gazebo_nav amcl.launch; \
# exec bash"

# sleep 5s 

# gnome-terminal --tab -- bash -c "\
# source devel/setup.bash; \
# roslaunch gazebo_nav racecar_nav_straight.launch; \
# exec bash"

# sleep 5s 

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

