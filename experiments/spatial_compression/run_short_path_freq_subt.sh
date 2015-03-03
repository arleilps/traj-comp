#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

mkdir SPFS/

for u in ${length_subt_vec[@]}
do
	for ((f=1; f<=$num_folds;f++))
	do
	#	echo "$traj_comp -c SPFS -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_train_$f -p $short_path -s $min_sup -u $u -f $conf_file > SPFS/train_train_$f\_$u"
	#	$traj_comp -c SPFS -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_train_$f -p $short_path -s $min_sup -u $u -f $conf_file > SPFS/train_train_$f\_$u
	
		echo "$traj_comp -c SPFS -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -p $short_path -s $min_sup -u $u -f $conf_file > SPFS/train_test_$f\_$u"
		$traj_comp -c SPFS -g $road_net -t $map_matched_traj\_train_$f -e $map_matched_traj\_test_$f -p $short_path -s $min_sup -u $u -f $conf_file > SPFS/train_test_$f\_$u
	done
done

