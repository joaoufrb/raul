#!/bin/bash

source /opt/ros/noetic/setup.bash

echo "Opening new terminal to ROS... "
xterm -xrm "xterm*allowTitleOps: false" -title "ROSCORE" -e "source /opt/ros/noetic/setup.bash; roscore" &

sleep 5

echo "Opening new terminal to Gazebo... "
xterm -xrm "xterm*allowTitleOps: false" -title "GAZEBO" -e "
	source /opt/ros/noetic/setup.bash 
	roslaunch jackal_gazebo hrtac_world.launch; sleep 30" &

sleep 5

echo "Opening new terminal to RVIZ... "
xterm -xrm "xterm*allowTitleOps: false" -title "RVIZ" -e "
	source /opt/ros/noetic/setup.bash 
	rviz " &


echo "Opening new terminal to RQT_GRAPH... "
xterm -xrm "xterm*allowTitleOps: false" -title "RQT_GRAPH" -e "
	source /opt/ros/noetic/setup.bash 
	rqt_graph " &

echo "Todos os terminais foram abertos. Digite qualquer tecla para fechar tudo."

# idle waiting for abort from user
#read -r -d '' _ </dev/tty
#wait
read

echo "Fechando todos os terminais com ROS..."
rosnode kill -a; 
sleep 5; 
killall roscore
killall xterm
sleep 5;
echo "Pronto!"

