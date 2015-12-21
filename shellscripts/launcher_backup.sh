



# Script for guaranteed safe start of data_collector
data_collectorXterm=
data_collectorProc=

while [[ -z "$data_collectorProc" ]]; do 
	
	xterm -e "rosrun experiment_runner data_collector" &
	data_collectorXterm=$!
	# echo "data_collectorXterm="$data_collectorXterm

	sleep 5s

	data_collectorProc=$(pidof data_collector)
	# echo "data_collectorProc="$data_collectorProc

	if test -z "$data_collectorProc"; then
		kill $data_collectorXterm
		sleep 5s
		kill -9 $data_collectorXterm
		sleep 5s
		echo "Had to force kill data_collector xterm...restarting it"
	fi
done



experiment_start_time=$(date +%s)
# echo "experiment_start_time---"$experiment_start_time

current_time=






while :; do


	sleep 10s

	gazeboProc=$(pidof gazebo)
	cmvisionProc=$(pidof cmvision)
	amclProc=$(pidof amcl)
	nestedAmclProc=$(pidof nested_amcl)
	rvizProc=$(pidof rviz)
	behaveProc=$(pidof behave | wc -w)
	floor1_robot2Proc=$(pidof floor1_robot2)
	floor1_robot1Proc=$(pidof floor1_robot1)
	data_collectorProc=$(pidof data_collector)

	current_time=$(date +%s)

	
	diff=$((current_time-experiment_start_time))
	#echo "diff ----"$diff
	
	
	if test -z "$nestedAmclProc"; then
		rostopic pub -1 /robot1_experiment_state std_msgs/String 'nestedAmclProc crashed'
		echo "nestedAmclProc crashed"
#		break
	elif test -z "$amclProc"; then
		rostopic pub -1 /robot1_experiment_state std_msgs/String 'amclProc crashed'
		echo "amclProc crashed"
#		break
	elif test $diff -gt "600"; then
		rostopic pub -1 /robot1_experiment_state std_msgs/String 'experiment timed out'
		echo "experiment timed out"
#		break
	elif test $((current_time-absolute_start_time)) -gt "900"; then
		rostopic pub -1 /robot1_experiment_state std_msgs/String 'shell timed out'
		echo "shell timed out"
#		break	
	elif test -z "$floor1_robot1Proc"; then
		rostopic pub -1 /robot1_experiment_state std_msgs/String 'floor1_robot1Proc crashed'
		echo "floor1_robot1Proc crashed"
#		break
	elif test -z "$floor1_robot2Proc"; then
		rostopic pub -1 /robot1_experiment_state std_msgs/String 'floor1_robot2Proc crashed'
		echo "floor1_robot2Proc crashed"
#		break
	elif test "$behaveProc" -lt 2; then
		rostopic pub -1 /robot1_experiment_state std_msgs/String 'behaveProc: one of the behaviours crashed'
		echo "behaveProc: one of the behaviours crashed"
#		break
	elif test -z "$rvizProc"; then
		rostopic pub -1 /robot1_experiment_state std_msgs/String 'rvizProc crashed'
		echo "rvizProc crashed"
#		break
	elif test -z "$cmvisionProc"; then
		rostopic pub -1 /robot1_experiment_state std_msgs/String 'cmvisionProc crashed'
		echo "cmvisionProc crashed"
#		break
	elif test -z "$gazeboProc"; then
		rostopic pub -1 /robot1_experiment_state std_msgs/String 'gazeboProc crashed'
		echo "gazeboProc crashed"
#		break
	fi

	if test -z "$data_collectorProc"; then
		echo "data_collectorProc ended(crash/shutdown)...stopping experiment"
		break
	fi
done


killall behave
sleep 2s
killall -9 behave
kill $nestedAmclProc $amclProc $floor1_robot1Proc $floor1_robot2Proc $rvizProc $cmvisionProc $gazeboProc
sleep 2s
kill -9 $nestedAmclProc $amclProc $floor1_robot1Proc $floor1_robot2Proc $rvizProc $cmvisionProc $gazeboProc


kill $amclXterm $floor1_robot1Xterm $floor1_robot2Xterm $behaveXterm $rvizXterm $cmvisionXterm $gazeboXterm
sleep 2s
kill -9 $amclXterm $floor1_robot1Xterm $floor1_robot2Xterm $behaveXterm $rvizXterm $cmvisionXterm $gazeboXterm
