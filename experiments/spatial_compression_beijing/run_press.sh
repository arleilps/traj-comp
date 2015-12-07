#!/bin/bash
#       Runs PRESS compression
#       @Arlei Silva

source settings.sh

mkdir SPFS/

echo "$traj_comp -c INDSP -g $road_net -f $conf_file -n $n_threads -p $short_path -l $short_path_file > SPFS/short_path_time"
#$traj_comp -c INDSP -g $road_net -f $conf_file -n $n_threads -p $short_path -l $short_path_file > SPFS/short_path_time

for a in ${delay_vec[@]}
do
	for ((f=1; f<=$num_folds;f++))
	do
		echo "$traj_comp -c SPFS -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -p $short_path -s $min_sup -u $length_subt_vec -f $conf_file -a $a -n $n_threads -l $short_path_file > SPFS/train_test_$f\_$a"
		$traj_comp -c SPFS -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -p $short_path -s $min_sup -u $length_subt_vec -f $conf_file -a $a -n $n_threads -l $short_path_file > SPFS/train_test_$f\_$a
	done
done
