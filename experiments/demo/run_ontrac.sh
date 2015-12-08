#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

rm -rf ONTRAC
mkdir ONTRAC/

for a in ${order_vec[@]}
do
	tfold=$trajectory_folder\_$a
	mkdir $tfold
	echo "$traj_comp -g $road_net -t $map_matched_traj\_train_1 -e $map_matched_traj\_test_1 -c ONTRAC-FULL -r $a -f $conf_file -m 60 -n $n_threads -o ONTRAC/output_1 -i $num_iter -q $queries\_test_1 -j $tfold >> ONTRAC/result_$a"
	$traj_comp -g $road_net -t $map_matched_traj\_train_1 -e $map_matched_traj\_test_1 -c ONTRAC-FULL -r $a -f $conf_file -m 60 -n $n_threads -o ONTRAC/output_1 -i $num_iter -q $queries\_test_1 -j $tfold >> ONTRAC/result_$a
done
