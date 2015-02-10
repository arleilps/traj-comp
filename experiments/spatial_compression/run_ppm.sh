#!/bin/bash
#       Runs PPM compression
#       @Arlei Silva

source settings.sh

mkdir PPM/
mkdir PPM/training_rate/

for t in ${training_rate_vec[@]}
do
	echo "$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -r $order > PPM/training_rate/train_train_$t"
	$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -r $order > PPM/training_rate/train_train_$t
	echo "$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -r $order > PPM/training_rate/train_test_$t"
	$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -r $order > PPM/training_rate/train_test_$t
done

mkdir PPM/order/

for d in ${order_vec[@]}
do
	echo "$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_train_$training_rate -r $d > PPM/order/train_train_$d"
	$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_train_$training_rate -r $d > PPM/order/train_train_$d
	echo "$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_test_$training_rate -r $d > PPM/order/train_test_$d"
	$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_test_$training_rate -r $d > PPM/order/train_test_$d
done

