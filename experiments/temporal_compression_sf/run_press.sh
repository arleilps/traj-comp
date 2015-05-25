#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

mkdir SPFS/

for a in ${delay_vec[@]}
do
	for ((f=1; f<=$num_folds;f++))
	do
		echo "$traj_comp -c NSTD -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -m $a > SPFS/train_test_$f\_$u\_$a"
		$traj_comp -c NSTD -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -m $a > SPFS/train_test_$f\_$u\_$a
	done
done
