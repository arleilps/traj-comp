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

results_file_name='SPFS_training_rate.dat'
rm $results_file_name

for t in ${training_rate_vec[@]}
do
	comp_ratio_train_train=$(extract "SPFS/training_rate/train_train_$t" "compression_ratio")
	comp_ratio_train_test=$(extract "SPFS/training_rate/train_test_$t" "compression_ratio")
	
	comp_time_train_train=$(extract "SPFS/training_rate/train_train_$t" "compression_time")
	comp_time_train_test=$(extract "SPFS/training_rate/train_test_$t" "compression_time")
	
	train_time_train_train=$(extract "SPFS/training_rate/train_train_$t" "training_time")
	train_time_train_test=$(extract "SPFS/training_rate/train_test_$t" "training_time")

	echo "$t	$comp_ratio_train_train	$comp_ratio_train_test	$comp_time_train_train	$train_time_train_train	$train_time_train_test" >> $results_file_name
done

