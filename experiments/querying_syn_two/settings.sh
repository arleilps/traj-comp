#!/bin/bash
#       Default parameters for experiments
#       @Arlei Silva


num_folds=1
length_subt_vec=10
order_vec=(2)
delay_vec=(30 60 120 240 480 960 1920)

min_sup=0
short_path=1000
n_threads=8
num_iter=5
map_matched_traj='../../../data/map_matched_syn_two'
road_net='../../../data/road_net_sfo.csv'
conf_file='../../../data/sfo_postgis.conf'
traj_comp='../../traj_comp'
short_path_file='../../../data/short_path_sfo'
queries="../../../data/queries_syn_two"
