source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://cpr-j100-0851:11311
export ROS_IP=`hostname -I | sed "s/ //"`
export LD_LIBRARY_PATH=~/libfreenect2/build/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu
