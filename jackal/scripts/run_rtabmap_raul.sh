#!/bin/bash

# How to enter in jackal computer:
# ssh administrator@192.168.0.101
# clearpath

git_dir="~/raul_workspace/raul"

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
	source $git_dir/jackal/scripts/raul_setup.bash; 
	roslaunch rplidar_ros rplidar_c1_raul.launch
	sleep 5" &

sleep 5

# Run Kinect2:
#Notice that the following command was edited: roslaunch kinect2_bridge kinect2_bridge_jackal.launch;
echo "Opening new terminal to Kinect Bridge... "
xterm -xrm "xterm*allowTitleOps: false" -title "KINECT BRIDGE" -e "
	source $git_dir/jackal/scripts/raul_setup.bash; 
        roslaunch kinect2_bridge kinect2_bridge_raul.launch publish_tf:=true depth_method:=cpu reg_method:=cpu;
	sleep 5" &
sleep 10

# View jackal in RViz:
#        roslaunch jackal_viz view_robot.launch;
echo "Opening new terminal to RVIZ... "
xterm -xrm "xterm*allowTitleOps: false" -title "RVIZ" -e "
	source $git_dir/jackal/scripts/raul_setup.bash; 
        rviz -d /home/joao/raul_workspace/raul/jackal/scripts/raul_all.rviz;
	sleep 5;" &
sleep 2


# Run Apriltag_ros:
# Se necessário alterar "_qhd" para "_sd" ou "_hd". 
echo "Opening new terminal to Apriltag_ros... "
xterm -xrm "xterm*allowTitleOps: false" -title "APRILTAG_ROS" -e "
	source /opt/ros/noetic/setup.bash 
	source ~/catkin_ws/devel/setup.bash
	source ~/remote-robot.sh
	roslaunch apriltag_ros continuous_detection_raul_qhd.launch; sleep 10" &

sleep 5

# Run Rtabmap:
echo "Opening new terminal to RTABMAP... "
xterm -xrm "xterm*allowTitleOps: false" -title "RTABMAP" -e "
	source $git_dir/jackal/scripts/raul_setup.bash; 
	roslaunch rtabmap_launch rtabmap_jackal.launch rtabmap_args:=\"--delete_db_on_start\";
	sleep 5" &

sleep 10

echo "Opening new terminal to RQT_GRAPH... "
xterm -xrm "xterm*allowTitleOps: false" -title "RQT_GRAPHP" -e "
	source $git_dir/jackal/scripts/raul_setup.bash;
	rqt_graph;
	sleep 5" &

