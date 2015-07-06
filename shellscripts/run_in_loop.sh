#!/bin/bash

run_count=0;

while test "$run_count" -lt "1000"; do
	./launcher.sh
	let "run_count = run_count + 1"
	echo "=========Experiment run_count: "$run_count
	sleep 20s

done

echo "reached the set run_count limit....ending this set of experiments."