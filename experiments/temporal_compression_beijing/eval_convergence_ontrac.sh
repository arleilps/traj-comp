#!/bin/bash
#       Evaluates PPM compression
#       @Arlei Silva

source settings.sh

$traj_comp -g ../data/road_net_sfo.csv -t ../data/map_matched_cab_stream_sfo.txt -e ../data/map_matched_cab_stream_sfo_test_1 -o conv_sf.dat -m 30 -f ../data/sfo_postgis.conf -n 8 -i 100 -c EMKF
