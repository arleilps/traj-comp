#!/bin/bash
#       Runs PRESS compression
#       @Arlei Silva

source settings.sh

mkdir NOCOMPRESSION/

for ((f=1; f<=$num_folds;f++))
do
	echo "$traj_comp -c NONE -g $road_net -e $map_matched_traj\_test_$f -f $conf_file -o output_$f -q $queries\_test_$f > NOCOMPRESSION/results_$f"
	$traj_comp -c NONE -g $road_net -e $map_matched_traj\_test_$f -f $conf_file -o output_$f -q $queries\_test_$f > NOCOMPRESSION/results_$f
done
