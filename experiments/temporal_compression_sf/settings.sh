#!/bin/bash
#       Default parameters for experiments
#       @Arlei Silva


num_folds=1
delay_vec=(30 60 120 240 480 960 1920)

n_threads=8

map_matched_traj='../../../data/map_matched_cab_stream_sfo'
road_net='../../../data/road_net_sfo.csv'
conf_file='../../../data/sfo_postgis.conf'
traj_comp='../../traj_comp'
