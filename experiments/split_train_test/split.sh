input_file_name=$1
output_file_name=$2
num_folds=3

cut -f1,1 -d, $input_file_name | sort -k1,1 -t, -u > traj.tmp
num_traj=`wc -l traj.tmp | awk '{print $1}'`
start="1"
fold_size=`echo "scale=10; $num_traj/$num_folds" | bc`
fold_size=`echo "$fold_size" | python -c "print int(round(float(raw_input())))"`
end=$fold_size

for ((s=1; s<=$num_folds;s++))
do
  sed -n $start,$end\p traj.tmp > fold_$s
  start=`echo "scale=1; $end+1" | bc`
  start=`echo "$start" | python -c "print int(round(float(raw_input())))"`
  end=`echo "scale=1; $end+$fold_size" | bc`
  end=`echo "$end" | python -c "print int(round(float(raw_input())))"`
done

rm traj.tmp

for ((s=1; s<=$num_folds;s++))
do
	rm $output_file_name\_test_$s
	rm $output_file_name\_train_$s
	
	for ((i=1; i<=$num_folds;i++))
	do
		if [ $s -eq $i ] 
		then
			grep -f fold_$i $input_file_name > $output_file_name\_test_$s
		else
			grep -f fold_$i $input_file_name >> $output_file_name\_train_$s
		fi
	done
done

for ((s=1; s<=$num_folds;s++))
do
	rm fold_$s
done
