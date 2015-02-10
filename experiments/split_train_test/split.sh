input_file_name=$1
output_file_name=$2
size_train=(5 10 20 40)
num_lines=`wc -l $input_file_name | awk '{print $1}'`
e=`echo "scale=10; $num_lines+1" | bc`

for s in ${size_train[@]}
do
  t=`echo "scale=10; $num_lines*$s/100" | bc`
  t=`echo "$t" | python -c "print int(round(float(raw_input())))"`
  t=`echo "scale=10; $t+1" | bc`
  sed -n 2,$t\p $input_file_name > $output_file_name\_train_$s
  t=`echo "scale=10; $t+1" | bc`
  sed -n $t,$e\p $input_file_name > $output_file_name\_test_$s
done
