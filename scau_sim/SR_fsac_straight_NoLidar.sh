#!/bin/bash

# 下面是无雷达无amcl简易方案的运行包顺序，需要进行对应代码的更改再运行

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

