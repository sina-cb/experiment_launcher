#! /bin/bash

c=0;

while [ $c -le 2 ]
do
	export RUN_NUMBER=$c
	echo "=========Experiment run_count: "$c
	/home/sina/indigo_workspace/src/experiment_launcher/shellscripts/launcher.sh
	c=$((c+1))
	sleep 5s
	echo "=========Experiment run_count: "$c" Finished..."
done

echo "Finished running exeriment!" | mail -s "Experiment Finished" s.solaimanpour@gmail.com