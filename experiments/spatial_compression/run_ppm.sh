#!/bin/bash
#       Runs PPM compression
#       @Arlei Silva

source settings.sh

mkdir PPM/

for d in ${order_vec[@]}
do
	for ((f=1; f<=$num_folds;f++))
	do
		echo "$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_train_$f -r $d -n $n_threads > PPM/train_train_$f\_$d"
		$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_train_$f -r $d -n $n_threads > PPM/train_train_$f\_$d
		echo "$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -r $d -n $n_threads > PPM/train_test_$f\_$d"
		$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -r $d -n $n_threads > PPM/train_test_$f\_$d
	done
done

