#!/bin/bash

#clear

absolute_start_time=$(date +%s)
# echo "absolute_start_time----"$absolute_start_time

#cd $HOME/electric_workspace/unversioned/turtlebot_exp/launch/floor_1_nested_experiments/
#xterm -hold -e "roscd turtlebot_exp/launch/floor_1_nested_experiments/" &

# Script for guaranteed safe start of gazebo
gazeboXterm=
gazeboProc=

while [ -z "$gazeboProc" ]; do 
	
	xterm -e "roslaunch experiment_launcher two_robots_gazebo_5th_floor_big.launch" &
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

while [ -z "$cmvisionProc" ]; do 
	
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
	
	xterm -e "roslaunch experiment_launcher amcl_multiple_robots_big.launch" &
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

while [ -z "$rvizProc" ]; do 
	
	xterm -e "roslaunch experiment_launcher view_navigation.launch" &
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

# Script for guaranteed safe start of both behave procs

behaveXterm=
behaveProc=

# while [ -z "$behaveProc" ]; do 
	
# 	xterm -e "roslaunch turtlebot_behaviour multi_robot_behaviour.launch" &
# 	behaveXterm=$!
# 	# echo "behaveXterm="$behaveXterm

# 	sleep 5s

# 	behaveProc=$(pidof behave | wc -w)
# 	# echo "behaveProc="$behaveProc

# 	if test "$behaveProc" -lt "2"; then
# 		kill $behaveXterm
# 		sleep 5s
# 		kill -9 $behaveXterm
# 		sleep 5s
# 		echo "Had to force kill behave xterm...restarting it"
# 		behaveProc=

# 	fi
# done


# Script for guaranteed safe start of floor5_robot1
floor5_robot1Xterm=
floor5_robot1Proc=

while [ -z "$floor5_robot1Proc" ]; do 
	
	xterm -e "rosrun experiment_launcher floor5_robot1" &
	floor5_robot1Xterm=$!
	# echo "floor5_robot1Xterm="$floor5_robot1Xterm

	sleep 2s

	floor5_robot1Proc=$(pidof floor5_robot1)
	# echo "floor5_robot1Proc="$floor5_robot1Proc

	if test -z "$floor5_robot1Proc"; then
		kill $floor5_robot1Xterm
		sleep 5s
		kill -9 $floor5_robot1Xterm
		sleep 5s
		echo "Had to force kill floor5_robot1 xterm...restarting it"
	fi
done


sleep 20s

# Script for guaranteed safe start of floor5_robot2
floor5_robot2Xterm=
floor5_robot2Proc=

while [ -z "$floor5_robot2Proc" ]; do 
	
	xterm -e "rosrun experiment_launcher floor5_robot2" &
	floor5_robot2Xterm=$!
	# echo "floor5_robot2Xterm="$floor5_robot2Xterm

	sleep 2s

	floor5_robot2Proc=$(pidof floor5_robot2)
	# echo "floor5_robot2Proc="$floor5_robot2Proc

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
	rvizProc=$(pidof rviz)
	behaveProc=$(pidof behave | wc -w)
	floor5_robot2Proc=$(pidof floor5_robot2)
	floor5_robot1Proc=$(pidof floor5_robot1)
	#data_collectorProc=$(pidof data_collector)

	current_time=$(date +%s)

	
	diff=$((current_time-experiment_start_time))
	#echo "diff ----"$diff
	echo $diff
	
	if test -z "$nestedAmclProc"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'nestedAmclProc crashed'
		echo "nestedAmclProc crashed"
#		break
	elif test -z "$amclProc"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'amclProc crashed'
		echo "amclProc crashed"
#		break
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
#	elif test "$behaveProc" -lt 2; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'behaveProc: one of the behaviours crashed'
#		echo "behaveProc: one of the behaviours crashed"
#		break
	elif test -z "$rvizProc"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'rvizProc crashed'
		echo "rvizProc crashed"
#		break
	elif test -z "$cmvisionProc"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'cmvisionProc crashed'
		echo "cmvisionProc crashed"
#		break
	elif test -z "$gazeboProc"; then
#		rostopic pub -1 /robot1_experiment_state std_msgs/String 'gazeboProc crashed'
		echo "gazeboProc crashed"
#		break
	fi

#	if test -z "$data_collectorProc"; then
#		echo "data_collectorProc ended(crash/shutdown)...stopping experiment"
#		break
#	fi

done


#killall behave
sleep 2s
#killall -9 behave
kill $nestedAmclProc $amclProc $floor5_robot1Proc $floor5_robot2Proc $rvizProc $cmvisionProc $gazeboProc
sleep 2s
kill -9 $nestedAmclProc $amclProc $floor5_robot1Proc $floor5_robot2Proc $rvizProc $cmvisionProc $gazeboProc


kill $amclXterm $floor5_robot1Xterm $floor5_robot2Xterm $behaveXterm $rvizXterm $cmvisionXterm $gazeboXterm
sleep 2s
kill -9 $amclXterm $floor5_robot1Xterm $floor5_robot2Xterm $behaveXterm $rvizXterm $cmvisionXterm $gazeboXterm
