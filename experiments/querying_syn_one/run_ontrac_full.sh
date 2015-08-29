#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

mkdir ONTRACFULL/

for a in ${delay_vec[@]}
do
	for ((f=1; f<=$num_folds;f++))
	do
		echo "$traj_comp -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -c ONTRAC-FULL -r 2 -f $conf_file -m $a -n $n_threads -o ouput_$f\_$a -i $num_iter -q $queries\_test_$f >> ONTRACFULL/result_$f\_$a"
		$traj_comp -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -c ONTRAC-FULL -r 2 -f $conf_file -m $a -n $n_threads -o ouput_$f\_$a -i $num_iter -q $queries\_test_$f >> ONTRACFULL/result_$f\_$a
	done
done
