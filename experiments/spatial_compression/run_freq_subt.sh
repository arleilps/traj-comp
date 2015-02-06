#!/bin/bash
#       Runs FST compression
#       @Arlei Silva

source settings.sh

for t in ${training_rate_vec[@]}
do
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $min_sup -u $length_subt >> fs_train_$t\_train_$t\_min_sup_$min_sup\_length_subt_$length_subt"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $min_sup -u $length_subt >> fs_train_$t\_train_$t\_min_sup_$min_sup\_length_subt_$length_subt
	
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -s $min_sup -u $length_subt >> fs_train_$t\_test_$t\_min_sup_$min_sup\_length_subt_$length_subt"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -s $min_sup -u $length_subt >> fs_train_$t\_test_$t\_min_sup_$min_sup\_length_subt_$length_subt
done

for u in ${length_subt_vec[@]}
do
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $min_sup -u $u >> fs_train_$t\_train_$t\_min_sup_$min_sup\_length_subt_$u"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $min_sup -u $u >> fs_train_$t\_train_$t\_min_sup_$min_sup\_length_subt_$u
	
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -s $min_sup -u $u >> fs_train_$t\_train_$t\_min_sup_$min_sup\_length_subt_$u"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -s $min_sup -u $u >> fs_train_$t\_test_$t\_min_sup_$min_sup\_length_subt_$u
done

for m in ${min_sup_vec[@]}
do
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $m -u $length_subt >> fs_train_$t\_train_$t\_min_sup_$m\_length_subt_$length_subt"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $m -u $length_subt >> fs_train_$t\_train_$t\_min_sup_$m\_length_subt_$length_subt
	
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_test\_train_$t -s $m -u $length_subt >> fs_train_$t\_train_$t\_min_sup_$m\_length_subt_$length_subt"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -s $m -u $length_subt >> fs_train_$t\_test_$t\_min_sup_$m\_length_subt_$length_subt
done

