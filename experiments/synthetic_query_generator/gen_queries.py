# -*- coding: utf-8 -*-
# <nbformat>3.0</nbformat>

# <codecell>

import random
import sys
import math
import csv

def read_trajectories(input_file_name):
	objs = []
	times = []

	input_file = open(input_file_name, 'r')
	for line in csv.reader(open(input_file_name).readlines()):
		obj = line[0]
		time = line[-2]

		objs.append(obj)
		times.append(time)

	return objs, times

def  generate_queries(objs, times, num_queries):
	queries = []

	for i in range(num_queries):
		r = random.randint(0, len(objs)-1)
		queries.append([objs[r], times[r]])

	return queries

def write_queries(queries):
	for q in queries:
		sys.stdout.write(str(q[0])+","+str(q[1])+"\n")

traj_file_name = sys.argv[1]
num_queries = 10000

objs, times = read_trajectories(traj_file_name)

queries = generate_queries(objs, times, num_queries)

write_queries(queries)
