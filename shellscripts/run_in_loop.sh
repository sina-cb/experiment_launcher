#!/bin/bash

run_count=0;

while test "$run_count" -lt "50"; do
	export RUN_NUMBER=$run_count
	echo "=========Experiment run_count: "$run_count
	/home/sina/indigo_workspace/src/experiment_launcher/shellscripts/launcher.sh
	let "run_count = run_count + 1"
	sleep 20s
done

echo "reached the set run_count limit....ending this set of experiments."