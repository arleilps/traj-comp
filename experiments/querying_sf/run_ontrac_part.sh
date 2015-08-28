#!/bin/bash
#       Runs SP compression
#       @Arlei Silva

source settings.sh

mkdir ONTRAC/

for a in ${delay_vec[@]}
do
	for ((f=1; f<=$num_folds;f++))
	do
		echo "./traj_comp -g $roat_net -t $map_matched_traj\_train_$ -e $map_matched_traj\_test_$f -c ONTRAC-FULL -r 2 -f ../data/sfo_postgis.conf -m $a -n $num_threads -o ouput_$f\_$a -i $num_iter -q $queries\_test_$f"
		./traj_comp -g $roat_net -t $map_matched_traj\_train_$ -e $map_matched_traj\_test_$f -c ONTRAC-FULL -r 2 -f ../data/sfo_postgis.conf -m $a -n $num_threads -o ouput_$f\_$a -i $num_iter -q $queries\_test_$f
	done
done
