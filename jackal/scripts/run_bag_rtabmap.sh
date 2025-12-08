#!/bin/bash

# How to enter in jackal computer:
# ssh administrator@192.168.0.101
# clearpath

git_dir="$HOME/raul_workspace/raul"

source "$git_dir/jackal/scripts/bagplay_setup.bash"

echo "NOTE: `hostname` has ROS_IP=$ROS_IP"

echo "Opening new terminal to ROSCORE... "
xterm -xrm "xterm*allowTitleOps: false" -title "ROSCORE" -e "
	source $git_dir/jackal/scripts/bagplay_setup.bash
	roscore;
	sleep 5" &
sleep 2

# View jackal in RViz:
#        roslaunch jackal_viz view_robot.launch;
echo "Opening new terminal to RVIZ... "
xterm -xrm "xterm*allowTitleOps: false" -title "RVIZ" -e "
	source $git_dir/jackal/scripts/bagplay_setup.bash
        rviz -d /home/joao/raul_workspace/raul/jackal/scripts/raul_all.rviz;
	sleep 5;" &
sleep 2


# Run Rtabmap:
echo "Opening new terminal to RTABMAP... "
xterm -xrm "xterm*allowTitleOps: false" -title "RTABMAP" -e "
	source $git_dir/jackal/scripts/bagplay_setup.bash
	roslaunch rtabmap_launch rtabmap_bag.launch rtabmap_args:=\"--delete_db_on_start\";
	sleep 5" &

sleep 10

echo "Opening new terminal to RQT_GRAPH... "
xterm -xrm "xterm*allowTitleOps: false" -title "RQT_GRAPHP" -e "
	source $git_dir/jackal/scripts/bagplay_setup.bash
	rqt_graph;
	sleep 5" &

echo "Opening new terminal to RQT_TF_TREE... "
xterm -xrm "xterm*allowTitleOps: false" -title "RQT_TF_TREE" -e "
	source $git_dir/jackal/scripts/bagplay_setup.bash
	rosrun rqt_tf_tree rqt_tf_tree;
	sleep 5" &

echo "Opening new terminal to ROSBAG... "
xterm -xrm "xterm*allowTitleOps: false" -title "ROSBAG" -e "
	source $git_dir/jackal/scripts/bagplay_setup.bash
	rosbag play $git_dir/../bags/2025-12-02-11-40-31.bag;
	sleep " 

