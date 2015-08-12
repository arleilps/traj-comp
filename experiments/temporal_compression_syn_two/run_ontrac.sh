#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

mkdir ONTRAC/

for ((f=1; f<=$num_folds;f++))
do
	echo "$traj_comp -c EM -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -m $a -f $conf_file -o conv_$f -i $num_iter -n $num_threads > ONTRAC/train_test_$f\_$a"
	$traj_comp -c EM -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -m $a -f $conf_file -o conv_$f -i $num_iter -n $num_threads > ONTRAC/train_test_$f\_$a
done
