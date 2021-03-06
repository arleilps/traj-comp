/**
Copyright (c) 2014, Arlei Silva
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author: Arlei Silva (arleilps@gmail.com)
**/

/**
 *	FILE test_moving_obj.cc: Test moving object functionalities
**/


/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>

/*my includes*/
#include "moving_obj.h"
#include "test_moving_obj.h"
#include "io.h"

const bool test_moving_obj()
{
	RoadNet* net = new RoadNet("road_net_sfo.csv");
	std::list<unsigned int> short_path;

	net->shortest_path(short_path, 0, 1);

	delete net;
	return true;
}

void read_matched_trajectories(std::map<unsigned int, std::string>& matched_traj, std::string input_file_name)
{
	std::ifstream input_file(input_file_name.c_str(), std::ios::in);
	std::string line_str;
	std::vector< std:: string > line_vec;
	std::getline(input_file, line_str);
	std::string id;
	std::string time;
	std::string edge;

	while(! input_file.eof())
	{
		line_vec = split(line_str,',');

		id = line_vec[0];
		time = line_vec[1];
		edge = line_vec[2];

		matched_traj[atoi(time.c_str())] = edge;
		
		std::getline(input_file, line_str);
	}

	input_file.close();
}

const bool test_moving_obj_file()
{
	/*Creating a road network*/
//	std::cout << "Creating a road network" << std::endl;
//	RoadNet* net = new RoadNet("../data/wa_adj.txt", "road_net_wa.csv");
//	delete net;
	
	RoadNet* net = new RoadNet("road_net_wa.csv");
	std::cout << "Road network created" << std::endl;

	std::map < unsigned int, std::string > matched_traj;
	double accuracy = 0;
	unsigned int num = 0;
	Trajectory* trajectory;
	std::vector<std::list< update* > * > updates;
	
	std::cout << "Map-matching updates" << std::endl;

	for(unsigned int i = 0; i < 1; i++)
	{
		Trajectory::read_updates(updates, "../data/wa_stream_" + to_string(i) + ".txt", net);
		trajectory = Trajectory::map_matching(*(updates.at(i)), net);
//		trajectory = Trajectory::read_trajectory("../data/wa_stream_" + to_string(i) + ".txt", net);
		matched_traj.clear();

		read_matched_trajectories(matched_traj, "../data/wa_matched_stream_" + to_string(i) + ".txt");
		
		for(Trajectory::iterator traj_it = trajectory->begin(); traj_it != trajectory->end(); ++traj_it)
		{
			if(matched_traj.at((*traj_it)->time) == net->seg_name((*traj_it)->segment))
			{
				accuracy = accuracy + 1;
			}
			
			num++;
		}
		
	//	trajectory->extend_traj_shortest_paths(net);
	//	trajectory->remove_repeated_segments();

		trajectory->print();

		delete trajectory;
		delete_list_updates(updates.at(i));
	}
	
	accuracy = (double) accuracy / num;

	std::cout << "accuracy = " << accuracy << std::endl;

	std::cout << "Deleting road network" << std::endl;
	delete net;
	std::cout << "Road network deleted" << std::endl;

	std::cout << "Test finished" << std::endl;

	return true;
}
	
const bool test_moving_obj_traj_file()
{
	/*Creating a road network*/
	std::cout << "Creating a road network" << std::endl;
//	RoadNet* net = new RoadNet("../data/wa_adj.txt", "road_net_wa.csv");
//	RoadNet* net = new RoadNet("road_net_wa.csv");
//	RoadNet* net = new RoadNet("../data/greater_sfo_adj.txt", "road_net_sf.csv");
	RoadNet* net = new RoadNet("road_net_sfo.csv");

	std::cout << "Road network created" << std::endl;

	std::cout << "Map-matching updates" << std::endl;

//	Trajectory::write_map_matched_trajectories(
//		"../data/wa_stream_small.txt", "map_matched.txt", net); 

	Trajectory::write_map_matched_trajectories(
		"../data/small_cab_stream_sfo.txt", "small_map_matched_cab_stream_sfo.txt", net); 
//	Trajectory::write_map_matched_trajectories(
//		"../data/cab_stream_sfo.txt", "map_matched_cab_stream_sfo.txt", net); 

	std::cout << "Deleting road network" << std::endl;
	delete net;
	std::cout << "Road network deleted" << std::endl;

	std::cout << "Test finished" << std::endl;

	return true;
}

const bool test_dist_times()
{
	Trajectory* traj = new Trajectory();
	RoadNet* net = new RoadNet("../data/road_net_sfo.csv");

	traj->add_update(0, 1, 1);
	traj->add_update(0, 0, 0);
	traj->add_update(0, 11, 11);
	traj->add_update(0, 0, 0);
	traj->add_update(0, 21, 21);

	std::list < dist_time* > dist_times;
	traj->get_dist_times_uniform(dist_times, net);

	for(std::list<dist_time*>::iterator it = dist_times.begin();
		it != dist_times.end(); ++it)
	{
		std::cout << "dist = " << (*it)->dist << " time = " << (*it)->time << std::endl;
	}

	return true;
}
