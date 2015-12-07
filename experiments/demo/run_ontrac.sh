#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

mkdir ONTRAC/

for a in ${order_vec[@]}
do
	tfold="$traj_folder\_$a"
	mkdir $tfold
	echo "$traj_comp -g $road_net -t $map_matched_traj\_train_0 -e $map_matched_traj\_test_0 -c ONTRAC-FULL -r 2 -f $conf_file -m 60 -n $n_threads -o ONTRAC/output_0 -i $num_iter -q $queries\_test_0 -j $tfold >> ONTRAC/result_0"
	$traj_comp -g $road_net -t $map_matched_traj\_train_0 -e $map_matched_traj\_test_0 -c ONTRAC-FULL -r 2 -f $conf_file -m 60 -n $n_threads -o ONTRAC/output_0 -i $num_iter -q $queries\_test_0 -j $tfold >> ONTRAC/result_0
done
