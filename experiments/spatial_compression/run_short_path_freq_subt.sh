#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

mkdir SPFS/
mkdir SPFS/training_rate/

for t in ${training_rate_vec[@]}
do
	echo "$traj_comp -c SPFS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -p $short_path -s $min_sup -u $length_subt -n $n_threads > SPFS/training_rate/train_train_$t"
	$traj_comp -c SPFS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -p $short_path -s $min_sup -u $length_subt -n $n_threads > SPFS/training_rate/train_test_$t
done

