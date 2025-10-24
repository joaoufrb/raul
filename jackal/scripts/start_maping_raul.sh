#!/bin/bash

source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash
source ~/remote-robot.sh

#Run Kinect_bridge
echo "Opening new terminal to kinect bridge... "
xterm -xrm "xterm*allowTitleOps: false" -title "kinect_bridge" -e "
	source /opt/ros/noetic/setup.bash 
	source ~/catkin_ws/devel/setup.bash
	source ~/remote-robot.sh
	roslaunch kinect2_bridge kinect2_bridge_jackal.launch; sleep 10" &

sleep 10

# Run Apriltag_ros:
# Se necessário alterar "_qhd" para "_sd" ou "_hd". 
echo "Opening new terminal to Apriltag_ros... "
xterm -xrm "xterm*allowTitleOps: false" -title "APRILTAG_ROS" -e "
	source /opt/ros/noetic/setup.bash 
	source ~/catkin_ws/devel/setup.bash
	source ~/remote-robot.sh
	roslaunch apriltag_ros continuous_detection_raul_qhd.launch; sleep 10" &

sleep 10

# Run Rtabmap:
echo "Opening new terminal to RTABMAP... "
xterm -xrm "xterm*allowTitleOps: false" -title "RTABMAP" -e "
	source /opt/ros/noetic/setup.bash 
	source ~/catkin_ws/devel/setup.bash
	source ~/remote-robot.sh
	roslaunch rtabmap_launch rtabmap_mapping_kinect2.launch depth_registration:=true rtabmap_args:=\"--delete_db_on_start\"; sleep 10" &

sleep 15

# Run Topic republish from /grid_map to /map
echo "Opening new terminal to map relay... "
xterm -xrm "xterm*allowTitleOps: false" -title "grid_map to map" -e "
	source /opt/ros/noetic/setup.bash 
	source ~/catkin_ws/devel/setup.bash
	source ~/remote-robot.sh
	rosrun topic_tools relay /rtabmap/grid_map /map; sleep 10" &

sleep 10

