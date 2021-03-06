#!/bin/bash
#       Runs PRESS compression
#       @Arlei Silva

source settings.sh

mkdir PRESS/

for a in ${delay_vec[@]}
do
	for ((f=1; f<=$num_folds;f++))
	do
		echo "$traj_comp -c NSTD -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -m $a -f $conf_file > PRESS/train_test_$f\_$a"
		$traj_comp -c NSTD -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -m $a -f $conf_file > PRESS/train_test_$f\_$a
	done
done
