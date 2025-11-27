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
	source $git_dir/jackal/scripts/raul_setup_ether.bash; 
	roslaunch rplidar_ros rplidar_c1_raul.launch
	sleep 5" &

sleep 5

# View jackal in RViz:
#        roslaunch jackal_viz view_robot.launch;
echo "Opening new terminal to RVIZ... "
xterm -xrm "xterm*allowTitleOps: false" -title "RVIZ" -e "
	source $git_dir/jackal/scripts/raul_setup_ether.bash; 
        rviz -d /home/joao/raul_workspace/raul/jackal/scripts/raul_all.rviz;
	sleep 5;" &
sleep 2


echo "Opening new terminal to RQT_GRAPH... "
xterm -xrm "xterm*allowTitleOps: false" -title "RQT_GRAPHP" -e "
	source $git_dir/jackal/scripts/raul_setup_ether.bash;
	rqt_graph;
	sleep 5" &

