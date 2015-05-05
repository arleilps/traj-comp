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

results_file_name='PPM_order.dat'
rm $results_file_name

for d in ${order_vec[@]}
do
	avg_comp_ratio_train_train=0
	avg_comp_ratio_train_test=0
	avg_comp_time_train_train=0
	avg_comp_time_train_test=0
	avg_train_time_train_train=0
	avg_train_time_train_test=0

	for ((f=1; f<=$num_folds;f++))
	do
#		comp_ratio_train_train=$(extract "PPM/train_train_$f\_$d" "compression_ratio")
#		avg_comp_ratio_train_train=`echo "scale=10; $avg_comp_ratio_train_train+$comp_ratio_train_train" | bc`
		
		comp_ratio_train_test=$(extract "PPM/train_test_${f}_${d}" "compression_ratio")
		avg_comp_ratio_train_test=`echo "scale=10; $avg_comp_ratio_train_test+$comp_ratio_train_test" | bc`
		
#		comp_time_train_train=$(extract "PPM/train_train_$f\_$d" "compression_time")
#		avg_comp_time_train_train=`echo "scale=10; $avg_comp_time_train_train+$comp_time_train_train" | bc`
		
		comp_time_train_test=$(extract "PPM/train_test_${f}_${d}" "compression_time")
		avg_comp_time_train_test=`echo "scale=10; $avg_comp_time_train_test+$comp_time_train_test" | bc`

#		train_time_train_train=$(extract "PPM/train_train_$f\_$d" "training_time")
#		avg_train_time_train_train=`echo "scale=10; $avg_comp_time_train_train+$train_time_train_train" | bc`
		
		train_time_train_test=$(extract "PPM/train_test_${f}_${d}" "training_time")
		avg_train_time_train_test=`echo "scale=10; $avg_train_time_train_test+$train_time_train_test" | bc`
	done
	
#	avg_comp_ratio_train_train=`echo "scale=10; $avg_comp_ratio_train_train/$num_folds" | bc`
	avg_comp_ratio_train_test=`echo "scale=10; $avg_comp_ratio_train_test/$num_folds" | bc`
	
#	avg_comp_time_train_train=`echo "scale=10; $avg_comp_time_train_train/$num_folds" | bc`
	avg_comp_time_train_test=`echo "scale=10; $avg_comp_time_train_test/$num_folds" | bc`
	
	avg_train_time_train_train=`echo "scale=10; $avg_train_time_train_train/$num_folds" | bc`
#	avg_train_time_train_test=`echo "scale=10; $avg_train_time_train_test/$num_folds" | bc`

	echo "$d	$avg_comp_ratio_train_train	$avg_comp_ratio_train_test	$avg_comp_time_train_train	$avg_comp_time_train_test	$avg_train_time_train_train	$avg_train_time_train_test" >> $results_file_name
done

