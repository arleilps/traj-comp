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

results_file_name='SP_training_rate.dat'
rm $results_file_name

for t in ${training_rate_vec[@]}
do
	comp_ratio_train_train=$(extract "SP/training_rate/train_train_$t" "compression_ratio")
	comp_ratio_train_test=$(extract "SP/training_rate/train_test_$t" "compression_ratio")
	
	comp_time_train_train=$(extract "SP/training_rate/train_train_$t" "compression_time")
	comp_time_train_test=$(extract "SP/training_rate/train_test_$t" "compression_time")
	
	train_time_train_train=$(extract "SP/training_rate/train_train_$t" "training_time")
	train_time_train_test=$(extract "SP/training_rate/train_test_$t" "training_time")

	echo "$t	$comp_ratio_train_train	$comp_ratio_train_test	$comp_time_train_train	$comp_time_train_test	$train_time_train_train	$train_time_train_test" >> $results_file_name
done

results_file_name='SP_short_path.dat'
rm $results_file_name

for p in ${short_path_vec[@]}
do
	comp_ratio_train_train=$(extract "SP/short_path/train_train_$p" "compression_ratio")
	comp_ratio_train_test=$(extract "SP/short_path/train_test_$p" "compression_ratio")
	
	comp_time_train_train=$(extract "SP/short_path/train_train_$p" "compression_time")
	comp_time_train_test=$(extract "SP/short_path/train_test_$p" "compression_time")
	
	train_time_train_train=$(extract "SP/short_path/train_train_$p" "training_time")
	train_time_train_test=$(extract "SP/short_path/train_test_$p" "training_time")

	echo "$p	$comp_ratio_train_train	$comp_ratio_train_test	$comp_time_train_train	$comp_time_train_test	$train_time_train_train	$train_time_train_test" >> $results_file_name
done

