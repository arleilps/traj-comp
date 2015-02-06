#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

for t in ${training_rate_vec[@]}
do
	echo "$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -p $short_path >> sp_train_$t\_train_$t\_short_path_$short_path"
	$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -p $short_path >> sp_train_$t\_train_$t\_short_path_$short_path
	echo "$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -p $short_path >> sp_train_$t\_test_$t\_short_path_$short_path"
	$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -p $short_path >> sp_train_$t\_test_$t\_short_path_$short_path
done

for p in ${short_path_vec[@]}
do
	echo "$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_train_$training_rate -p $p >> sp_train_$training_rate\_train_$training_rate\_short_path_$p"
	$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_train_$training_rate -p $p >> sp_train_$training_rate\_train_$training_rate\_short_path_$p
	echo "$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_test_$training_rate -p $p >> sp_train_$training_rate\_test_$training_rate\_short_path_$p"
	$traj_comp -c SP -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_test_$training_rate -p $p >> sp_train_$training_rate\_test_$training_rate\_short_path_$p
done

