#!/bin/bash

# How to enter in jackal computer:
# ssh administrator@192.168.0.101
# clearpath

git_dir="$HOME/raul_workspace/raul"

source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash

#export ROS_MASTER_URI=http://192.168.0.101:11311
export ROS_MASTER_URI=http://cpr-j100-0851:11311
#export ROS_IP=`hostname -I`

export ROS_IP=`hostname -I | sed "s/ .*//"`

echo "NOTE: `hostname` has ROS_IP=$ROS_IP"

# Run RPLIDAR C1:
echo "Opening new terminal to LIDAR... "
xterm -xrm "xterm*allowTitleOps: false" -title "LIDAR" -e "
	source $git_dir/jackal/scripts/raul_setup_ether.bash 
	roslaunch rplidar_ros rplidar_c1_raul.launch
	sleep 5" &

sleep 5

# Run Kinect2:
#Notice that the following command was edited: roslaunch kinect2_bridge kinect2_bridge_jackal.launch;
echo "Opening new terminal to Kinect Bridge... "
xterm -xrm "xterm*allowTitleOps: false" -title "KINECT BRIDGE" -e "
	source $git_dir/jackal/scripts/raul_setup_ether.bash; 
        roslaunch kinect2_bridge kinect2_bridge_raul.launch publish_tf:=true depth_method:=cpu reg_method:=cpu;
	sleep 5" &
sleep 10

# Run Apriltag_ros:
# Se necessário alterar "_qhd" para "_sd" ou "_hd". 
echo "Opening new terminal to Apriltag_ros... "
xterm -xrm "xterm*allowTitleOps: false" -title "APRILTAG_ROS" -e "
	source $git_dir/jackal/scripts/raul_setup_ether.bash; 
	roslaunch apriltag_ros continuous_detection_raul_qhd.launch; sleep 10" &

sleep 5

# Record topics:
echo "Opening new terminal to ROSBAG "
xterm -xrm "xterm*allowTitleOps: false" -title "ROSBAG" -e "
	source $git_dir/jackal/scripts/raul_setup_ether.bash; 
        rosbag record $(cat $git_dir/jackal/scripts/raul_topics.txt)
	sleep 10;" &

echo "Opening new terminal to RVIZ... "
xterm -xrm "xterm*allowTitleOps: false" -title "RVIZ" -e "
	source /opt/ros/noetic/setup.bash 
	rviz -d $git_dir/jackal/scripts/raul_all.rviz" &


sleep 5




