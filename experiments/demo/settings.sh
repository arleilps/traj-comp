#!/bin/bash
#       Default parameters for experiments
#       @Arlei Silva


num_folds=1
length_subt_vec=10
order_vec=(1 2 4 8)
delay_vec=(0 30 60 120 240 480)

min_sup=0
short_path=1000
n_threads=8
num_iter=5
map_matched_traj='../../../data/map_matched_cab_stream_sfo'
road_net='../../../data/road_net_sfo.csv'
conf_file='../../../data/sfo_postgis.conf'
traj_comp='../../traj_comp'
short_path_file='../../../data/short_path_sfo'
queries="../../../data/queries_sfo"
trajectory_folder="traj_folder"
