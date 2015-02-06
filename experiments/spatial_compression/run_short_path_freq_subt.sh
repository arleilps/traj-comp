#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

for t in ${training_rate_vec[@]}
do
	echo "$traj_comp -c SPFS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -p $short_path -s $min_sup -u $length_subt >> spfs_train_$t\_train_$t\_short_path_$short_path\_min_sup_$min_sup\_length_subt_$length_subt"
	$traj_comp -c SPFS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -p $short_path -s $min_sup -u $length_subt >> spfs_train_$t\_train_$t\_short_path_$short_path\_min_sup_$min_sup\_length_subt_$length_subt
done

