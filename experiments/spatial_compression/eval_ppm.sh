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

results_file_name='PPM_training_rate.dat'
rm $results_file_name

for t in ${training_rate_vec[@]}
do
	comp_ratio_train_train=$(extract "PPM/training_rate/train_train_$t" "compression_ratio")
	comp_ratio_train_test=$(extract "PPM/training_rate/train_test_$t" "compression_ratio")
	
	comp_time_train_train=$(extract "PPM/training_rate/train_train_$t" "compression_time")
	comp_time_train_test=$(extract "PPM/training_rate/train_test_$t" "compression_time")
	
	train_time_train_train=$(extract "PPM/training_rate/train_train_$t" "training_time")
	train_time_train_test=$(extract "PPM/training_rate/train_test_$t" "training_time")

	echo "$t	$comp_ratio_train_train	$comp_ratio_train_test	$comp_time_train_train	$train_time_train_train	$train_time_train_test" >> $results_file_name
done

results_file_name='PPM_order.dat'
rm $results_file_name

for d in ${order_vec[@]}
do
	comp_ratio_train_train=$(extract "PPM/order/train_train_$d" "compression_ratio")
	comp_ratio_train_test=$(extract "PPM/order/train_test_$d" "compression_ratio")
	
	comp_time_train_train=$(extract "PPM/order/train_train_$d" "compression_time")
	comp_time_train_test=$(extract "PPM/order/train_test_$d" "compression_time")
	
	train_time_train_train=$(extract "PPM/order/train_train_$d" "training_time")
	train_time_train_test=$(extract "PPM/order/train_test_$d" "training_time")

	echo "$d	$comp_ratio_train_train	$comp_ratio_train_test	$comp_time_train_train	$train_time_train_train	$train_time_train_test" >> $results_file_name
done

