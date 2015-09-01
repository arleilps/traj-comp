#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

mkdir ONTRAC/

for ((f=1; f<=$num_folds;f++))
do
	echo "$traj_comp -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -c ONTRAC-FULL -r 2 -f $conf_file -m 60 -n $n_threads -o ONTRAC/output_$f -i $num_iter -q $queries\_test_$f >> ONTRAC/result_$f"
	$traj_comp -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -c ONTRAC-FULL -r 2 -f $conf_file -m 60 -n $n_threads -o ONTRAC/output_$f -i $num_iter -q $queries\_test_$f >> ONTRAC/result_$f
done
