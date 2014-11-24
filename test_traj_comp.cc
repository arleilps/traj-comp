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
 *	FILE test_traj_comp.cc: Testing trajectory compression
**/


/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>

/*my includes*/
#include "test_traj_comp.h"
#include "traj_comp.h"

const bool test_traj_comp_freq_subt()
{
	std::cout << "Testing frequent sub-trajectory based compression" << std::endl;

	RoadNet* net = new RoadNet("road_net_sf.csv");
	FreqSubt* fs = new  FreqSubt(2, 100, net);
	
	Trajectory* traj;

	/*0, 1, 2, 3, 4*/
	traj = new Trajectory();
	//An update is a pair segment id, timestamp, but here the timestamps don't matter.
	traj->add_update(0, 0);
	traj->add_update(1, 1);
	traj->add_update(2, 2);
	traj->add_update(3, 3);
	traj->add_update(4, 4);

	fs->add_trajectory(traj);
	
	/*2, 3, 4*/
	traj = new Trajectory();
	traj->add_update(2, 0);
	traj->add_update(3, 1);
	traj->add_update(4, 2);
	
	fs->add_trajectory(traj);
	
	/*2, 3*/
	traj = new Trajectory();
	traj->add_update(2, 1);
	traj->add_update(3, 2);
	
	fs->add_trajectory(traj);

	/*4*/
	traj = new Trajectory();
	traj->add_update(4, 0);
	
	fs->add_trajectory(traj);
	
	/*4, 3, 2*/
	traj = new Trajectory();
	traj->add_update(4, 3);
	traj->add_update(3, 4);
	traj->add_update(2, 5);
	
	fs->add_trajectory(traj);

	//Printing the tree and the frequency of the respective substrings
	fs->print();

	//Gets all frequent sub-trajectories (minimum support 2)
	std::list<Trajectory*> fsts;
	fs->freq_sub_traj(fsts);

	for(std::list<Trajectory*>::iterator it = fsts.begin(); it != fsts.end(); ++it)
	{
		traj = *it;
		
		//Prints a trajectory
		for(Trajectory::iterator traj_it = traj->begin(); traj_it != traj->end(); ++traj_it)
		{
			std::cout << "<seg: " << (*traj_it)->segment <<
				" time: " << (*traj_it)->time << ">" << std::endl;
		}

		std::cout << std::endl;
	}
/*
	std::cout << "Deleting road network";
	delete net;
	std::cout << "Road network deleted";
*/	
	std::cout << "Test finished" << std::endl;
	return true;
}

	
const bool test_traj_comp_freq_subt_sf_cab()
{
	/*
	std::cout << "Creating a road network" << std::endl;
	//RoadNet* net = new RoadNet("../data/greater_sfo_adj.txt", "road_net_sf.csv");
	RoadNet* net = new RoadNet("road_net_sf.csv");
	std::cout << "Road network created" << std::endl;
	 std::list<unsigned int> short_path;
	 std::cout << "Map-matching updates" << std::endl;
	 Trajectory::write_map_matched_trajectories(
	 	"../data/cab_stream_sfo.txt", "map_matched_cab_stream_sfo.txt", net);
	 	//"../data/small_cab_stream_sfo.txt", "map_matched_cab_stream_sfo.txt", net);

	std::cout << "Deleting road network" << std::endl;
	delete net;
	std::cout << "Road network deleted" << std::endl;
	
	std::cout << "Test finished" << std::endl;
	*/

	RoadNet* net = new RoadNet("road_net_sf.csv");
//	RoadNet* net = new RoadNet("road_net.csv");

	FreqSubt* fs = new  FreqSubt(10, 4, net);
	std::cout << "size_tree = " << fs->train("map_matched_cab_stream_sfo.txt") << std::endl;;
	
	std::list<Trajectory*> fsts;
	fs->freq_sub_traj(fsts);
	Trajectory* traj;
	
	for(std::list<Trajectory*>::iterator it = fsts.begin(); it != fsts.end(); ++it)
	{
		traj = *it;
		
		//Prints a trajectory
		for(Trajectory::iterator traj_it = traj->begin(); traj_it != traj->end(); ++traj_it)
		{
			std::cout << net->seg_name((*traj_it)->segment) << " ";
		}

		std::cout << std::endl;
	}
	
	//fs->train("map_matched.txt");
	//std::cout << "num_updates = " << fs->test("map_matched.txt") << std::endl;
	std::cout << "num_updates = " << fs->test("map_matched_cab_stream_sfo.txt") << std::endl;

	delete fs;

	return true;
}
