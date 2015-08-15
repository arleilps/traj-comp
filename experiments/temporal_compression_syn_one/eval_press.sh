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

results_file_name='press_error.dat'
rm $results_file_name

for d in ${delay_vec[@]}
do
	avg_comp_ratio_train_test=0
	avg_comp_time_train_test=0
	avg_train_time_train_test=0

	for ((f=1; f<=$num_folds;f++))
	do
		comp_ratio_train_test=$(extract "PRESS/train_test_${f}_${d}" "compression_ratio")
		avg_comp_ratio_train_test=`echo "scale=10; $avg_comp_ratio_train_test+$comp_ratio_train_test" | bc`
		
		comp_time_train_test=$(extract "PRESS/train_test_${f}_${d}" "compression_time")
		avg_comp_time_train_test=`echo "scale=10; $avg_comp_time_train_test+$comp_time_train_test" | bc`

		train_time_train_test=$(extract "PRESS/train_test_${f}_${d}" "training_time")
		avg_train_time_train_test=`echo "scale=10; $avg_train_time_train_test+$train_time_train_test" | bc`
	done
	
	avg_comp_ratio_train_test=`echo "scale=10; $avg_comp_ratio_train_test/$num_folds" | bc`
	avg_comp_time_train_test=`echo "scale=10; $avg_comp_time_train_test/$num_folds" | bc`
	avg_train_time_train_test=`echo "scale=10; $avg_train_time_train_test/$num_folds" | bc`

	echo "$d        $avg_comp_ratio_train_test" >> $results_file_name
done

results_file_name='press_time.dat'
rm $results_file_name

echo "0        $avg_train_time_train_test" >> $results_file_name
echo "0        $avg_comp_time_train_test" >> $results_file_name

