input_file_name=$1
output_file_name=$2
num_folds=10

num_lines=`wc -l $input_file_name | awk '{print $1}'`
line="1"
for ((s=1; s<=$num_folds;s++))
do
  fold_size=`echo "scale=10; $num_lines/$num_folds" | bc`
  fold_size=`echo "$fold_size" | python -c "print int(round(float(raw_input())))"`
  sed -n $line,$fold_size\p $input_file_name > fold_$s
  line=`echo "scale=10; $line+$fold_size" | bc`
  line=`echo "$line" | python -c "print int(round(float(raw_input())))"`
done

for ((s=1; s<=$num_folds;s++))
do
	rm $output_file_name\_test_$s
	rm $output_file_name\_train_$s
	
	for ((i=1; i<=$num_folds;i++))
	do
		if [ $s -eq $i ] 
		then
			cat fold_$i > $output_file_name\_test_$s
		else
			cat fold_$i >> $output_file_name\_train_$s
		fi
	done
done

for ((s=1; s<=$num_folds;s++))
do
	rm fold_$s
done
