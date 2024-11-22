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

export ROS_IP=`hostname -I | sed "s/ //"`

# View jackal in RViz:
echo "Opening new terminal to RVIZ... "
xterm -xrm "xterm*allowTitleOps: false" -title "RVIZ" -e "
	source $git_dir/jackal/scripts/raul_setup.bash; 
        roslaunch jackal_viz view_robot.launch" &

sleep 15

# Run Kinect2:
echo "Opening new terminal to Kinect Bridge... "
xterm -xrm "xterm*allowTitleOps: false" -title "KINECT BRIDGE" -e "
	source $git_dir/jackal/scripts/raul_setup.bash; 
        roslaunch kinect2_bridge kinect2_bridge_jackal.launch" &

sleep 15

# Run Rtabmap:
echo "Opening new terminal to RTABMAP... "
xterm -xrm "xterm*allowTitleOps: false" -title "RTABMAP" -e "
	source $git_dir/jackal/scripts/raul_setup.bash; 
	roslaunch rtabmap_launch rtabmap_jackal.launch rtabmap_args:=\"--delete_db_on_start\"" &

sleep 15

echo "Opening new terminal to RQT_GRAPH... "
xterm -xrm "xterm*allowTitleOps: false" -title "RQT_GRAPHP" -e "
	source $git_dir/jackal/scripts/raul_setup.bash;
	rqt_graph" &


