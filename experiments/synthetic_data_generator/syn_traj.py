# -*- coding: utf-8 -*-
# <nbformat>3.0</nbformat>

# <codecell>

import random
import sys
import math
import networkx

def build_road_network(input_file_name, avg_seg_length):
    input_file = open(input_file_name, 'r')
    net = networkx.DiGraph()
    seg_lengths = {}
    num_vertices = int(input_file.readline())
    seg_ids = {}
  
    for i in range(0, num_vertices):
        line = input_file.readline()
        vec = line.rsplit(',')
	seg_lengths[int(vec[0])] = float(vec[-2])
	seg_ids[int(vec[0])] = vec[-1]
 
    for j in range(0, num_vertices):
        line = input_file.readline()
        vec = line.rsplit(',')
 
        for i in range(1, len(vec)):
            net.add_edge(int(vec[0]), int(vec[i]))
  
    input_file.close()
    
    return net, seg_lengths, seg_ids

def smooth_speeds_traj(speeds, window_size):
    new_speeds = []
    for s in range(0, len(speeds)):
        avg = 0
        num = 0
        for t in range(s-window_size, s+window_size+1):
            if t >= 0 and t < len(speeds):
                avg = avg + speeds[t] 
                num = num + 1
        
        avg = float(avg) / num
        new_speeds.append(avg)
        
    return new_speeds        

def generate_speed_dist(net, avg_speed, std_speed_all, std_speed_seg):
    avg_speeds = {}
    sig_speeds = {}
    
    for s in net.nodes():
        avg_speeds[s] = max(5, math.fabs(random.gauss(avg_speed, std_speed_all)))
        sig_speeds[s] = math.fabs(random.gauss(0, std_speed_seg))
    
    return avg_speeds, sig_speeds

def generate_trajectories2(num_trajs, road_network, lambd):
    pois = []
    probs = []
    trajs = []
    
    for i in road_network.nodes():
        pois.append(i)

    random.shuffle(pois)

    for i in range(num_trajs):
        
	poi = int(random.expovariate(lambd))
        
	src = len(road_network.nodes())
	
	while src >= len(road_network.nodes()):
		src = random.randint(0,len(road_network.nodes())-1)
        
	try:
            traj = networkx.shortest_path(road_net, source=src,target=poi)
        except Exception:
            traj = []
        
        if len(traj) > 0:
            trajs.append(traj)
            
    return trajs

def generate_trajectories(num_trajs, num_segs, road_network):
    trajs = []
    while True:
        traj=[road_network.nodes()[random.randint(0,len(road_network.nodes())-1)]]
        for s in range(num_segs-1):
            num = 0
            if len(road_network.neighbors(traj[-1])) > 0:
                next_node = road_network.neighbors(traj[-1])[random.randint(0, len(road_network.neighbors(traj[-1]))-1)]
            
                while next_node in traj and num < 16:
                    next_node = road_network.neighbors(traj[-1])[random.randint(0, len(road_network.neighbors(traj[-1]))-1)]
                    num = num + 1
            
                if num < 16:
                    traj.append(next_node)
        
        if len(traj) >= num_segs:
            trajs.append(traj)
            
        if len(trajs) >= num_trajs:
            break
    
    return trajs

def generate_speeds(trajs, avg_speeds, sig_speeds, num_smooth, window):
    speeds = []
    
    for i in range(0, len(trajs)):
        traj = trajs[i]
                   
        spds = []
        
        for j in range(0, len(traj)):
            v = max(5, random.gauss(avg_speeds[traj[j]], sig_speeds[traj[j]]))
            spds.append(v)
        
        for n in range(0, num_smooth):
            spds = smooth_speeds_traj(spds, window)
        
        speeds.append(spds)
        
    return speeds

def generate_updates(trajs, speeds, sigma, gap, seg_lengths):
    updates = []
    for t in range(0, len(trajs)):
        traj = trajs[t]
        spds = speeds[t]
        
        ups = []
        segs = []
        time = 0
        dist = 0
        time_last = 0
        
        for s in range(0, len(traj)):
            dist = dist + seg_lengths[traj[s]]
            time = time + float(seg_lengths[traj[s]]) / spds[s]
            segs.append(traj[s])
            
            if time - time_last >= gap:
                if len(ups) > 0:
                    speed = float(dist - ups[-1][2]) / (time - ups[-1][1])
                else:
                    speed = float(dist) / time
                time_with_error = math.fabs(random.gauss(time, float(sigma) / speed))
                ups.append([segs, time, dist])
                time_last = time
                segs = []
                
        if len(segs) > 0:
            ups.append([segs, time, dist])
            
        updates.append(ups)
        
    return updates

def compute_sig_trans(speeds, trajs, seg_lengths):
    sig_trans = 0
    num = 0
    for i in range(0, len(speeds)):
        spds = speeds[i]
        traj = trajs[i]
        
        for v in range(1, len(spds)):
            sig_trans = sig_trans + math.pow(float(seg_lengths[traj[v]])/spds[v]-float(seg_lengths[traj[v-1]])/spds[v-1], 2)
            num = num + 1
    
    sig_trans = math.sqrt(float(sig_trans) / num)
    
    return sig_trans

def compute_avg_sig_speed(speeds):
    avg_speed = 0
    sig_speed = 0
    num = 0
    
    for i in range(0, len(speeds)):
        spds = speeds[i]
        
        for v in range(0, len(spds)):
            avg_speed = avg_speed + spds[v]
            num = num + 1
    
    avg_speed = float(avg_speed) / num
    
    for i in range(0, len(speeds)):
        spds = speeds[i]
        
        for v in range(0, len(spds)):
            sig_speed = sig_speed + math.pow(spds[v] - avg_speed, 2)
            
    sig_speed = math.sqrt(float(sig_speed) / num)
    
    return avg_speed, sig_speed


def compute_avg_sig_times(speeds, trajs, net, seg_lengths):
    avg_times = {}
    sig_times = {}
    nums = {}
    
    for v in net.nodes():
        avg_times[v] = 0
        sig_times[v] = 0
        nums[v] = 0
    
    for i in range(0, len(speeds)):
        spds = speeds[i]
        traj = trajs[i]
        
        for v in range(0, len(spds)):
            avg_times[traj[v]] = avg_times[traj[v]] + float(seg_lengths[traj[v]]) / spds[v]
            nums[traj[v]] = nums[traj[v]] + 1
    
    for v in net.nodes():
        if nums[v] > 0:
            avg_times[v] = float(avg_times[v]) / nums[v]
    
    for i in range(0, len(speeds)):
        spds = speeds[i]
        traj = trajs[i]
        
        for v in range(0, len(spds)):
            sig_times[traj[v]] = sig_times[traj[v]] + math.pow(avg_times[traj[v]]-float(seg_lengths[traj[v]]) / spds[v], 2)
    
    for v in net.nodes():
        if nums[v] > 0:
            sig_times[v] = math.sqrt(float(sig_times[v]) / nums[v]) + math.sqrt(sys.float_info.epsilon)
        else:
            sig_times[v] = math.sqrt(sys.float_info.epsilon)
    return avg_times, sig_times

def compute_avg_sig_times2(times, trajs, net):
    avg_times = {}
    sig_times = {}
    nums = {}
    
    for v in net.nodes():
        avg_times[v] = 0
        sig_times[v] = 0
        nums[v] = 0
    
    for i in range(0, len(times)):
        tms = times[i]
        traj = trajs[i]
        
        for v in range(0, len(tms)):
            avg_times[traj[v]] = avg_times[traj[v]] + tms[v]
            nums[traj[v]] = nums[traj[v]] + 1
    
    for v in net.nodes():
        if nums[v] > 0:
            avg_times[v] = float(avg_times[v]) / nums[v]
    
    for i in range(0, len(times)):
        tms = times[i]
        traj = trajs[i]
        
        for v in range(0, len(tms)):
            sig_times[traj[v]] = sig_times[traj[v]] + math.pow(avg_times[traj[v]]-tms[v], 2)
    
    for v in net.nodes():
        if nums[v] > 0:
            sig_times[v] = math.sqrt(float(sig_times[v]) / nums[v]) + math.sqrt(sys.float_info.epsilon)
        else:
            sig_times[v] = math.sqrt(sys.float_info.epsilon)
            
    return avg_times, sig_times

def compute_travel_times(speeds, trajs, seg_lengths):
    times = []
    
    for i in range(0, len(speeds)):
        spds = speeds[i]
        traj = trajs[i]
        tms = []
        
        for s in range(len(spds)):
            t = float(seg_lengths[traj[s]]) / spds[s]
            tms.append(t)
            
        times.append(tms)
            
    return times

def compute_travel_time_ups(updates, seg_lengths):
    times = []
    
    for u in range(len(updates)):
        ups = updates[u]
        tms = []
        
        for i in range(len(ups)):
            if i > 0:
                t = float(ups[i][1]-ups[i-1][1]) / len(ups[i][0])
            else:
                t = float(ups[i][1]) / len(ups[i][0])

            for j in range(len(ups[i][0])):
                tms.append(t)
        
        times.append(tms)
    
    return times

def write_road_net(roadnet, seg_lengths, seg_ids, output_file_name):
    output_file = open(output_file_name, 'w')
    
    output_file.write(str(len(roadnet.nodes()))+"\n")
    
    for v in roadnet.nodes():
        output_file.write(str(v)+",0,1,1,1,1.4255e+07,1.41256e+07,5.46093e+06,5.49709e+06,0,"+str(seg_lengths[v])+","+seg_ids[v]+"\n")
        
    for v in roadnet.nodes():
        output_file.write(str(v))
        for n in roadnet.neighbors(v):
            if v < n:
                output_file.write(","+str(n))
        output_file.write("\n")
    
    output_file.close()

def write_updates(updates, seg_ids, output_file_name, seg_lengths):
    output_file = open(output_file_name, 'w')
    
    for i in range(0, len(updates)):
        ups = updates[i]
        output_file.write(str(i)+","+seg_ids[ups[0][0][0]]+",1,0\n")
        
        for j in range(0, len(ups)):
            if j == 0:
                if len(ups[j][0]) > 1:    
                    output_file.write(str(i))
                    
                for s in range(1, len(ups[j][0])):
                    output_file.write(","+seg_ids[ups[j][0][s]])
            
                if len(ups[j][0]) > 1:
                    output_file.write(","+str(int(ups[j][1]))+","+str(seg_lengths[ups[j][0][-1]])+"\n")
            else:
                output_file.write(str(i))
                for s in range(0, len(ups[j][0])):
                    output_file.write(","+seg_ids[ups[j][0][s]])
                output_file.write(","+str(int(ups[j][1]))+","+str(seg_lengths[ups[j][0][-1]])+"\n")

    output_file.close()            

sig_gps = 10
time_updates = 480
num_trajectories = 500000
lambd = 0.00001

#sig_gps = 5
#time_updates = 120
#num_trajectories = 500000
#lambd = 1

(road_net, seg_lengths, seg_ids) = build_road_network("/home/arlei/trajectory_compression/data/road_net_sfo.csv", 100)
(avg_speeds, sig_speeds) = generate_speed_dist(road_net, 15, 10, 5)
trajs = generate_trajectories2(num_trajectories, road_net, lambd)
speeds = generate_speeds(trajs, avg_speeds, sig_speeds, 2, 2) 
ups = generate_updates(trajs, speeds, sig_gps, time_updates, seg_lengths)

write_road_net(road_net, seg_lengths, seg_ids, "road_net_syn_two.csv")
write_updates(ups, seg_ids, "map_matched_syn_two.csv", seg_lengths)

#write_road_net(road_net, seg_lengths, seg_ids, "road_net_syn_one.csv")
#write_updates(ups, seg_ids, "map_matched_syn_one.csv", seg_lengths)
