#!/bin/bash
#       Evaluates PPM compression
#       @Arlei Silva

source settings.sh

extract (){
	file=$1
	measure=$2

	value=`grep -m1 $measure $file | cut -d ' ' -f3`
	value=`echo ${value} | sed -e 's/[eE]+*/\\*10\\^/'`
	
	echo $value
}

results_file_name='FS_training_rate.dat'
rm $results_file_name

for t in ${training_rate_vec[@]}
do
	comp_ratio_train_train=$(extract "FS/training_rate/train_train_$t" "compression_ratio")
	comp_ratio_train_test=$(extract "FS/training_rate/train_test_$t" "compression_ratio")
	
	comp_time_train_train=$(extract "FS/training_rate/train_train_$t" "compression_time")
	comp_time_train_test=$(extract "FS/training_rate/train_test_$t" "compression_time")
	
	train_time_train_train=$(extract "FS/training_rate/train_train_$t" "training_time")
	train_time_train_test=$(extract "FS/training_rate/train_test_$t" "training_time")

	echo "$t	$comp_ratio_train_train	$comp_ratio_train_test	$comp_time_train_train	$train_time_train_train	$train_time_train_test" >> $results_file_name
done

results_file_name='FS_min_sup.dat'
rm $results_file_name

for m in ${min_sup_vec[@]}
do
	comp_ratio_train_train=$(extract "FS/min_sup/train_train_$m" "compression_ratio")
	comp_ratio_train_test=$(extract "FS/min_sup/train_test_$m" "compression_ratio")
	
	comp_time_train_train=$(extract "FS/min_sup/train_train_$m" "compression_time")
	comp_time_train_test=$(extract "FS/min_sup/train_test_$m" "compression_time")
	
	train_time_train_train=$(extract "FS/min_sup/train_train_$m" "training_time")
	train_time_train_test=$(extract "FS/min_sup/train_test_$m" "training_time")

	echo "$m	$comp_ratio_train_train	$comp_ratio_train_test	$comp_time_train_train	$train_time_train_train	$train_time_train_test" >> $results_file_name
done

results_file_name='FS_length_subt.dat'
rm $results_file_name

for u in ${length_subt_vec[@]}
do
	comp_ratio_train_train=$(extract "FS/length_subt/train_train_$u" "compression_ratio")
	comp_ratio_train_test=$(extract "FS/length_subt/train_test_$u" "compression_ratio")
	
	comp_time_train_train=$(extract "FS/length_subt/train_train_$u" "compression_time")
	comp_time_train_test=$(extract "FS/length_subt/train_test_$u" "compression_time")
	
	train_time_train_train=$(extract "FS/length_subt/train_train_$u" "training_time")
	train_time_train_test=$(extract "FS/length_subt/train_test_$u" "training_time")

	echo "$u	$comp_ratio_train_train	$comp_ratio_train_test	$comp_time_train_train	$train_time_train_train	$train_time_train_test" >> $results_file_name
done
