#!/bin/bash

#clear

absolute_start_time=$(date +%s)
# echo "absolute_start_time----"$absolute_start_time

# Script for guaranteed safe start of gazebo
gazeboXterm=
gazeboProc=

echo "Running Gazebo"
while [ -z "$gazeboProc" ]; do 
	
	xterm -e "roslaunch experiment_launcher two_robots_gazebo_5th_floor.launch" &
	gazeboXterm=$!

	sleep 15s

	gazeboProc=$(pidof gzclient)

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

echo "Running cmvision"
while [ -z "$cmvisionProc" ]; do 
	
	xterm -e "roslaunch experiment_launcher cmvision_node.launch" &
	cmvisionXterm=$!

	sleep 5s

	cmvisionProc=$(pidof cmvision)

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
echo "Running AMCL"
while :; do 
	xterm -e "roslaunch experiment_launcher amcl_multiple_robots_no_traj.launch" &
	amclXterm=$!

	sleep 25s

	amclProc=$(pidof amcl)
	nestedAmclProc=$(pidof nested_amcl)

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


# Script for guaranteed safe start of floor5_robot1
floor5_robot1Xterm=
floor5_robot1Proc=
echo "Running Robot1 Proc"
while [ -z "$floor5_robot1Proc" ]; do 
	
	xterm -e "rosrun experiment_launcher floor5_robot1" &
	floor5_robot1Xterm=$!

	sleep 2s

	floor5_robot1Proc=$(pidof floor5_robot1)

	if test -z "$floor5_robot1Proc"; then
		kill $floor5_robot1Xterm
		sleep 5s
		kill -9 $floor5_robot1Xterm
		sleep 5s
		echo "Had to force kill floor5_robot1 xterm...restarting it"
	fi
done

echo "Sleeping for 20 seconds"
sleep 20s

# Script for guaranteed safe start of floor5_robot2
floor5_robot2Xterm=
floor5_robot2Proc=

echo "Running Robot2 Proc"
while [ -z "$floor5_robot2Proc" ]; do 
	
	xterm -e "rosrun experiment_launcher floor5_robot2" &
	floor5_robot2Xterm=$!

	sleep 2s

	floor5_robot2Proc=$(pidof floor5_robot2)

	if test -z "$floor5_robot2Proc"; then
		kill $floor5_robot2Xterm
		sleep 5s
		kill -9 $floor5_robot2Xterm
		sleep 5s
		echo "Had to force kill floor5_robot2 xterm...restarting it"
	fi
done

experiment_start_time=$(date +%s)

while :; do
	sleep 10s

	gazeboProc=$(pidof gzclient)
	cmvisionProc=$(pidof cmvision)
	amclProc=$(pidof amcl)
	nestedAmclProc=$(pidof nested_amcl)
	floor5_robot2Proc=$(pidof floor5_robot2)
	floor5_robot1Proc=$(pidof floor5_robot1)

	current_time=$(date +%s)

	
	diff=$((current_time-experiment_start_time))
	#echo "diff ----"$diff
	
	if test -z "$nestedAmclProc"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'nestedAmclProc crashed'
		echo "nestedAmclProc crashed"
		break
	elif test -z "$amclProc"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'amclProc crashed'
		echo "amclProc crashed"
		break
	elif test $diff -gt "1200"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'experiment timed out'
		echo "experiment timed out"
		break
	elif test $((current_time-absolute_start_time)) -gt "1200"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'shell timed out'
		echo "shell timed out"
		break	
	elif test -z "$floor5_robot1Proc"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'floor5_robot1Proc crashed'
		echo "floor5_robot1Proc crashed"
		break
	elif test -z "$floor5_robot2Proc"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'floor5_robot2Proc crashed'
		echo "floor5_robot2Proc finished or crashed"
		break
	elif test -z "$cmvisionProc"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'cmvisionProc crashed'
		echo "cmvisionProc crashed"
		break
	elif test -z "$gazeboProc"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'gazeboProc crashed'
		echo "gazeboProc crashed"
		break
	fi

done


echo "Killing processes"
sleep 2s
kill $nestedAmclProc $amclProc $floor5_robot1Proc $floor5_robot2Proc $cmvisionProc $gazeboProc
sleep 2s
kill -9 $nestedAmclProc $amclProc $floor5_robot1Proc $floor5_robot2Proc $cmvisionProc $gazeboProc


kill $amclXterm $floor5_robot1Xterm $floor5_robot2Xterm $behaveXterm $cmvisionXterm $gazeboXterm
sleep 2s
kill -9 $amclXterm $floor5_robot1Xterm $floor5_robot2Xterm $behaveXterm $cmvisionXterm $gazeboXterm
