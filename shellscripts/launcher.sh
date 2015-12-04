#!/bin/bash

#clear

absolute_start_time=$(date +%s)
# echo "absolute_start_time----"$absolute_start_time

#cd $HOME/electric_workspace/unversioned/turtlebot_exp/launch/floor_1_nested_experiments/
#xterm -hold -e "roscd turtlebot_exp/launch/floor_1_nested_experiments/" &

# Script for guaranteed safe start of gazebo
gazeboXterm=
gazeboProc=

while [[ -z "$gazeboProc" ]]; do 
	
	xterm -e "roslaunch experiment_launcher two_robots_simulation.launch" &
	gazeboXterm=$!
	# echo "gazeboXterm="$gazeboXterm

	sleep 15s

	gazeboProc=$(pidof gzclient)
	# echo "gazeboProc="$gazeboProc

	if test -z "$gazeboProc"; then
		kill $gazeboXterm
		sleep 5s
		kill -9 $gazeboXterm
		sleep 5s
		echo "Had to force kill gazebo xterm...restarting it"
	fi
done

# Script for guaranteed safe start of cmvision
cmvisionXterm=
cmvisionProc=

while [[ -z "$cmvisionProc" ]]; do 
	
	xterm -e "roslaunch experiment_launcher cmvision_node.launch" &
	cmvisionXterm=$!
	# echo "cmvisionXterm="$cmvisionXterm

	sleep 5s

	cmvisionProc=$(pidof cmvision)
	# echo "cmvisionProc="$cmvisionProc

	if test -z "$cmvisionProc"; then
		kill $cmvisionXterm
		sleep 5s
		kill -9 $cmvisionXterm
		sleep 5s
		echo "Had to force kill cmvision xterm...restarting it"
	fi
done


# Script for guaranteed safe start of amcl and nested_amcl
amclXterm=
amclProc=
nestedAmclProc=

while :; do 
	
	xterm -e "roslaunch experiment_launcher amcl_multiple_robots.launch" &
	amclXterm=$!
	# echo "amclXterm="$amclXterm

	sleep 25s

	amclProc=$(pidof amcl)
	# echo "amclProc="$amclProc
	nestedAmclProc=$(pidof nested_amcl)
	# echo "nestedAmclProc="$nestedAmclProc

	if test -z "$amclProc"; then
		kill $amclXterm
		sleep 5s
		kill -9 $amclXterm
		sleep 5s
		echo "Had to force kill amcl xterm...restarting it"

	elif test -z "$nestedAmclProc"; then
		kill $amclXterm
		sleep 5s
		kill -9 $amclXterm
		sleep 5s
		echo "Had to force kill nested_amcl xterm...restarting it"

	else
		break

	fi
done

# Script for guaranteed safe start of rviz
rvizXterm=
rvizProc=

while [[ -z "$rvizProc" ]]; do 
	
	xterm -e "roslaunch experiment_launcher multi_nested_rviz.launch" &
	rvizXterm=$!
	# echo "rvizXterm="$rvizXterm

	sleep 15s

	rvizProc=$(pidof rviz)
	# echo "rvizProc="$rvizProc

	if test -z "$rvizProc"; then
		kill $rvizXterm
		sleep 5s
		kill -9 $rvizXterm
		sleep 5s
		echo "Had to force kill rviz xterm...restarting it"
	fi
done