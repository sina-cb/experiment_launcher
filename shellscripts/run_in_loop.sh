#! /bin/bash

c=0;

while [ $c -le 15 ]
do
	export RUN_NUMBER=$c
	echo "=========Experiment run_count: "$c
	#/home/sina/indigo_workspace/src/experiment_launcher/shellscripts/launcher.sh
	c=$((c+1))
	sleep 20s
	echo $c
done

echo "reached the set run_count limit....ending this set of experiments."