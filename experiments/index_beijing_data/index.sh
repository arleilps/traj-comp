PGPASSWORD="traj_comp"
sudo -u postgres createdb -O traj_comp beijing
sudo -u postgres psql -c "CREATE EXTENSION postgis; CREATE EXTENSION postgis_topology;" beijing
../../traj_comp -g ../../../data/beijing_adj.txt -c IND -o ../../../data/road_net_beijing.csv -f ../../../data/beijing_postgis.conf
