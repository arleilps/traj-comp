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
	traj->add_update(0, 0, 0);
	traj->add_update(1, 1, 1);
	traj->add_update(2, 2, 2);
	traj->add_update(3, 3, 3);
	traj->add_update(4, 4, 4);

	fs->add_trajectory(traj);
	
	/*2, 3, 4*/
	traj = new Trajectory();
	traj->add_update(2, 0, 0);
	traj->add_update(3, 1, 1);
	traj->add_update(4, 2, 2);
	
	fs->add_trajectory(traj);
	
	/*2, 3*/
	traj = new Trajectory();
	traj->add_update(2, 1, 1);
	traj->add_update(3, 2, 2);
	
	fs->add_trajectory(traj);

	/*4*/
	traj = new Trajectory();
	traj->add_update(4, 0, 0);
	
	fs->add_trajectory(traj);
	
	/*4, 3, 2*/
	traj = new Trajectory();
	traj->add_update(4, 3, 3);
	traj->add_update(3, 4, 4);
	traj->add_update(2, 5, 5);
	
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
				" time: " << (*traj_it)->start_time << ">" << std::endl;
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

const bool create_road_net()
{
	RoadNet* net = new RoadNet("../data/greater_sfo_adj.txt", "road_net_sf.csv");
	delete net;

	return true;
}

const bool map_matching()
{
	RoadNet* net = new RoadNet("road_net_sf.csv");
	Trajectory::write_map_matched_trajectories(
	 	"../data/cab_stream_sfo.txt", "map_matched_cab_stream_sfo.txt", net);
	
	delete net;

	return true;
}

const bool freq_sub_traj_queries()
{
	RoadNet* net = new RoadNet("road_net_sf.csv");
	
	//Frequent subtrajectories of size at most two
	//and frequency at most 10
	TrajDB* db = new FreqSubtCompTrajDB("map_matched_cab_stream_sfo.txt", 10, 2, net);
	
	//Use db->drop(); to erase the content of the database
	//db->drop(); 
	db->create();
	
	//This will insert all the trajectories in the file
	//into the database
	db->insert("map_matched_cab_stream_sfo.txt");

	//Some statistics
	std::cout << "#original updates: " << db->updates() << std::endl;
	std::cout << "#database updates:" << db->db_updates() << std::endl;

	std::list<std::string> q_results;

	//center = (lat,long), distance in meters
	//Accepts time range arguments (optional) not tested yet
	db->center_radius_query(37.78,-122.39, 100, q_results);

	//Printing query results
	for(std::list<std::string>::iterator it = q_results.begin();
		it != q_results.end(); it++)
	{
		std::cout << *it << std::endl;
	}

	delete net;
	delete db;
	return true;
}
	
const bool test_traj_comp_freq_subt_sf_cab()
{
	//Indexes the segments and creates
	//a file road_net_sf.csv, needs to
	//be executed only once.
//	create_road_net();

	//Map-matches the trajectory updates
	//Might take too long, depends on 
	//files and databases created in 
	//the previous step. Needs to be
	//executed only once.
//	map_matching();

	//Creating trajectory database
	//and performing queries using
	//frequent subtrajectories
	freq_sub_traj_queries();

	return true;
}
