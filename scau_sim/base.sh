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


echo “successfully started!”
