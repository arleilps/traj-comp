#!/bin/bash
#       Runs FST compression
#       @Arlei Silva

source settings.sh

mkdir FS/
mkdir FS/training_rate/

for t in ${training_rate_vec[@]}
do
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $min_sup -u $length_subt > FS/training_rate/train_train_$t"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $min_sup -u $length_subt > FS/training_rate/train_train_$t
	
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -s $min_sup -u $length_subt > FS/training_rate/train_test_$t"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -s $min_sup -u $length_subt > FS/training_rate/train_test_$t
done

mkdir FS/length_subt/

for u in ${length_subt_vec[@]}
do
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $min_sup -u $u > FS/length_subt/train_train_$u"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $min_sup -u $u > FS/length_subt/train_train_$u
	
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -s $min_sup -u $u > FS/length_subt/train_test_$u"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -s $min_sup -u $u > FS/length_subt/train_test_$u
done

mkdir FS/min_sup/

for m in ${min_sup_vec[@]}
do
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $m -u $length_subt > FS/min_sup/train_train_$m"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -s $m -u $length_subt >  FS/min_sup/train_train_$m
	
	echo "$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_test\_train_$t -s $m -u $length_subt > FS/min_sup/train_train_$m"
	$traj_comp -c FS -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -s $m -u $length_subt > FS/min_sup/train_test_$m
done

