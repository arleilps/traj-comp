#!/bin/bash
#       Default parameters for experiments
#       @Arlei Silva


num_folds=10
length_subt_vec=(10)
order_vec=(1 2 3 4 5 6)
delay_vec=(30 60 120 240 480 960 1920)

min_sup=0
short_path=100
n_threads=8

map_matched_traj='../../../data/map_matched_cab_stream_beijing'
road_net='../../../data/road_net_beijing.csv'
conf_file='../../../data/beijing_postgis.conf'
traj_comp='../../traj_comp'
