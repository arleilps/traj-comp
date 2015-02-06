#!/bin/bash
#       Runs PPM compression
#       @Arlei Silva

source settings.sh

for t in ${training_rate_vec[@]}
do
	echo "$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -r $order >> ppm_train_$t\_train_$t\_order_$order"
	$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_train_$t -r $order >> ppm_train_$t\_train_$t\_order_$order
	echo "$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -r $order >> ppm_train_$t\_test_$t\_order_$order"
	$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$t -e $map_matched_traj\_test_$t -r $order >> ppm_train_$t\_test_$t\_order_$order
done

for d in ${order_vec[@]}
do
	echo "$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_train_$training_rate -r $d >> ppm_train_$training_rate\_train_$training_rate\_order_$d"
	$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_train_$training_rate -r $d >> ppm_train_$training_rate\_train_$training_rate\_order_$d
	echo "$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_test_$training_rate -r $d >> ppm_train_$training_rate\_test_$training_rate\_order_$d"
	$traj_comp -c PPM -g $road_net -t $map_matched_traj\_train_$training_rate -e $map_matched_traj\_test_$training_rate -r $d >> ppm_train_$training_rate\_test_$training_rate\_order_$d
done

