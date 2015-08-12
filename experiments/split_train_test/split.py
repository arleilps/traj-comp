import sys

num_folds = 10
input_file_name = sys.argv[1]
output_file_name = sys.argv[2]

input_file = open(input_file_name, 'r')

folds  = {}
i = 0
for line in input_file:
	line = line.rstrip()
	vec = line.rsplit(',')
	
	if vec[0] not in folds:
		folds[vec[0]] = i % num_folds
		i = i + 1

input_file.close()

for i in range(num_folds):
	input_file = open(input_file_name, 'r')
	train_file = open(output_file_name + "_train_" + str(i), 'w')
	test_file = open(output_file_name + "_test_" + str(i), 'w')

	for line in input_file:
		line = line.rstrip()
		vec = line.rsplit(',')

		if folds[vec[0]] == i:
			test_file.write(line+"\n")
		else:
			train_file.write(line+"\n")

	train_file.close()
	test_file.close()

	input_file.close()

