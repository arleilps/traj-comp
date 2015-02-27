#!/bin/bash
#       Default parameters for experiments
#       @Arlei Silva


num_folds=10
length_subt_vec=(1 2 4 6 8 10)
order_vec=(1 2 3 4 5 6)

training_rate=10
min_sup=0
short_path=1600
n_threads=6

map_matched_traj='../../../data/map_matched_cab_stream_sfo'
road_net='../../../data/road_net_sfo.csv'
conf_file='../../../data/sfo_postgis.conf'
traj_comp='../../traj_comp'
