#!/bin/bash

source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash

#Run Kinect_bridge
echo "Opening new terminal to kinect bridge... "
xterm -xrm "xterm*allowTitleOps: false" -title "kinect_bridge" -e "
	source /opt/ros/noetic/setup.bash 
	source ~/catkin_ws/devel/setup.bash
	roslaunch kinect2_bridge kinect2_bridge_jackal.launch; sleep 10" &

sleep 10


# Run Apriltag_ros:
echo "Opening new terminal to Apriltag_ros... "
xterm -xrm "xterm*allowTitleOps: false" -title "APRILTAG_ROS" -e "
	source /opt/ros/noetic/setup.bash 
	source ~/catkin_ws/devel/setup.bash
	roslaunch apriltag_ros continuous_detection_raul.launch camera_name:=kinect2/sd image_topic:=image_color_rect; sleep 10" &

sleep 10

# Run jackal_viz:
echo "Opening new terminal to RViz... "
xterm -xrm "xterm*allowTitleOps: false" -title "JACKAL_VIZ" -e "
	source /opt/ros/noetic/setup.bash 
	source ~/catkin_ws/devel/setup.bash
	roslaunch jackal_viz view_robot.launch; sleep 10" &

sleep 10
# Run Rtabmap:
echo "Opening new terminal to RTABMAP... "
xterm -xrm "xterm*allowTitleOps: false" -title "RTABMAP" -e "
	source /opt/ros/noetic/setup.bash 
	source ~/catkin_ws/devel/setup.bash
	roslaunch rtabmap_launch rtabmap_jackal_teste.launch rtabmap_args:=\"--delete_db_on_start\" cfg:=~/.rtabmap/rtabmap.ini; sleep 10" &

sleep 15
