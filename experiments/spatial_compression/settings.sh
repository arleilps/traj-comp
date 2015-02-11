#!/bin/bash
#       Default parameters for experiments
#       @Arlei Silva


training_rate_vec=(5 10 20 40)
length_subt_vec=(2 4 6 8 10)
min_sup_vec=(1 10 100 1000)
short_path_vec=(200 400 800 1600 3200)
order_vec=(1 2 3 4 5 6)

training_rate=10
length_subt=6
min_sup=100
short_path=1600
order=2
n_threads=6

map_matched_traj='../../../data/map_matched_cab_stream_sfo'
road_net='../../../data/road_net_sfo.csv'

traj_comp='../../traj_comp'
