#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

mkdir SP/
mkdir SP/training_rate/

for t in ${training_rate_vec[@]}
do
	echo "$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -p $short_path -n $n_threads > SP/training_rate/train_train_$t"
	$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -p $short_path -n $n_threads > SP/training_rate/train_train_$t
	echo "$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -p $short_path -n $n_threads > SP/training_rate/train_test_$t"
	$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -p $short_path -n $n_threads > SP/training_rate/train_test_$t
done

mkdir SP/short_path/

for p in ${short_path_vec[@]}
do
	echo "$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_train_$training_rate -p $p -n $n_threads > SP/short_path/train_train_$p"
	$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_train_$training_rate -p $p -n $n_threads > SP/short_path/train_train_$p
	echo "$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_test_$training_rate -p $p -n $n_threads > SP/short_path/train_test_$p"
	$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_test_$training_rate -p $p -n $n_threads > SP/short_path/train_test_$p
done

