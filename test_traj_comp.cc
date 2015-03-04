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
	RoadNet* net = new RoadNet("../data/road_net_sfo.csv");
	TrajCompAlgo* traj_comp = new  FreqSubt(2, 5, net);
	
	traj_comp->train("map_matched_cab_stream_sfo.txt");
	traj_comp->test("map_matched_cab_stream_sfo.txt");

	delete traj_comp;
	delete net;

	
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
/*
void print_freq_sub_traj(FreqSubtCompTrajDB* db)
{
	std::list<std::pair<unsigned int, Trajectory*>*> fsts;
	db->freq_sub_traj(fsts);
	
	Trajectory* traj;
	
	for(std::list<std::pair<unsigned int, Trajectory* > * >::iterator it = fsts.begin(); 
		it != fsts.end(); ++it)
	{
		traj = (*it)->second;
		std::cout << (*it)->first << " : ";
		//Prints a trajectory
		for(Trajectory::iterator traj_it = traj->begin(); traj_it != traj->end(); ++traj_it)
		{
			std::cout << (*traj_it)->segment << " ";
		}

		std::cout << std::endl;

		delete traj;
		delete *it;
	}
}
*/
//const bool freq_sub_traj_queries()
//{
//	RoadNet* net = new RoadNet("road_net_sf.csv");
	
	//Frequent subtrajectories of size at most two
	//and frequency at most 10
//	FreqSubtCompTrajDB* db = new FreqSubtCompTrajDB("map_matched_cab_stream_sfo.txt", 10, 4, net);
	
	//Printing frequent subtrajectories with respective ids
//	print_freq_sub_traj(db);

	//Use db->drop(); to erase the content of the database
	//db->drop(); 
	//db->create();
	
	//This will insert all the trajectories in the file
	//into the database
	//db->insert("map_matched_cab_stream_sfo.txt");

	//Some statistics
	//std::cout << "#original updates: " << db->updates() << std::endl;
	//std::cout << "#database updates: " << db->db_updates() << std::endl;

//	std::list<std::string> q_results;

	//center = (lat,long), distance in meters
	//Accepts time range arguments (optional) not tested yet
//	db->center_radius_query(37.78,-122.39, 100, q_results);

	//Printing query results
//	for(std::list<std::string>::iterator it = q_results.begin();
//		it != q_results.end(); it++)
//	{
//		std::cout << *it << std::endl;
//	}

//	delete net;
//	delete db;
//	return true;
//}
	
//const bool test_traj_comp_freq_subt_sf_cab()
//{
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
//	freq_sub_traj_queries();

//	return true;
//}

const bool test_traj_comp_short_path()
{
//	create_road_net();
//	RoadNet* net = new RoadNet("../data/adj.txt", "road_net.csv");
//	delete net;
	
//	RoadNet* net = new RoadNet("road_net.csv");

//	Trajectory::write_map_matched_trajectories(
//	 	"../data/cab_stream.txt", "map_matched_cab_stream.txt", net);

//	TrajCompAlgo* traj_comp = new ShortestPath(net, std::numeric_limits<double>::max());
	
//	std::cout << traj_comp->test("map_matched_cab_stream.txt") << std::endl;
	
//	delete traj_comp;
	
//	delete net;

	RoadNet* net = new RoadNet("road_net_sf.csv");
	Trajectory::write_map_matched_trajectories(
	 	"../data/cab_stream_sfo.txt", "map_matched_cab_stream_sfo.txt", net);
	
//	TrajCompAlgo* traj_comp = new ShortestPath(net, pow(10, 2));

//	delete traj_comp;
	delete net;

	return true;
}

const bool test_traj_comp_short_path_freq_subt()
{
	RoadNet* net = new RoadNet("road_net.csv");

	//TrajCompAlgo* traj_comp = new FreqSubt(2, 100, net);
	//TrajCompAlgo* traj_comp = new ShortestPath(net, std::numeric_limits<double>::max());
	TrajCompAlgo* traj_comp = new ShortestPathFreqSubt(2, 100, std::numeric_limits<double>::max(), net, 1);
	
	traj_comp->train("map_matched_cab_stream.txt");

	traj_comp->test("map_matched_cab_stream.txt");
	
	delete traj_comp;
	
	delete net;

	return true;
}

const bool test_traj_comp_ppm()
{
	RoadNet* net = new RoadNet("road_net.csv");
	TrajCompAlgo* traj_comp = new PredPartMatch(10, net);

	traj_comp->train("map_matched_cab_stream_sfo.txt");
	//traj_comp->train("map_matched.txt");
	
	traj_comp->test("map_matched_cab_stream_sfo.txt");
	//unsigned int n = traj_comp->test("map_matched.txt");

	delete traj_comp;
	delete net;

	return true;
}

const bool test_least_squares()
{
	//RoadNet* net = new RoadNet("../data/road_net_sfo.csv");
	RoadNet* net = new RoadNet("../data/road_net.csv");
	TrajCompAlgo* traj_comp = new LeastSquares(10, 0.1, net);
	//TrajCompAlgo* traj_comp = new NSTD(net, 0);
	//traj_comp->train("../data/map_matched_cab_stream_sfo.txt");
	traj_comp->train("../data/map_matched_cab_stream.txt");
	traj_comp->test("../data/map_matched_cab_stream.txt");

	std::cout << "num updates = " << traj_comp->num_updates_comp() << std::endl;

	delete traj_comp;
	delete net;

	return true;
}

const bool test_TSND()
{
	/*
	angle R;
	dist_time p_index;
	dist_time p_i;

	R.from = (double) 0;
	R.to = (double) PI / 4;
	
	p_index.dist = 0;
	p_index.time = 0;

	p_i.dist = 0.99;
	p_i.time = 100;
	
	std::cout << TSND::fall_inside(R, p_index, p_i) << std::endl;

	p_index.dist = 10;
	p_index.time = 10;

	std::cout << TSND::fall_inside(R, p_index, p_i) << std::endl;

	p_i.dist = 99;
	p_i.time = 100;
	
	std::cout << TSND::fall_inside(R, p_index, p_i) << std::endl;

	std::cout << "R: " << R.from << " " << R.to << std::endl;
	
	TSND::constrain(R, p_index, p_i, 0);

	std::cout << "R: " << R.from << " " << R.to << std::endl;

	R.from = (double) 0;
	R.to = (double) PI / 4;
	
	std::cout << "R: " << R.from << " " << R.to << std::endl;
	
	TSND::constrain(R, p_index, p_i, 100000);
	
	std::cout << "R: " << R.from << " " << R.to << std::endl;
	*/

	std::list < dist_time* > dist_times;
	std::list < dist_time* > comp_dist_times;
	
	dist_time* dt = new dist_time;
	dt->dist = 0;
	dt->time = 0;
	dist_times.push_back(dt);
	
	dt = new dist_time;
	dt->dist = 0;
	dt->time = 1;
	dist_times.push_back(dt);

	dt = new dist_time;
	dt->dist = 0;
	dt->time = 2;
	dist_times.push_back(dt);

	dt = new dist_time;
	dt->dist = 1;
	dt->time = 3;
	dist_times.push_back(dt);

	dt = new dist_time;
	dt->dist = 2;
	dt->time = 4;
	dist_times.push_back(dt);
	
	dt = new dist_time;
	dt->dist = 3;
	dt->time = 5;
	dist_times.push_back(dt);
	
	dt = new dist_time;
	dt->dist = 3;
	dt->time = 6;
	dist_times.push_back(dt);
	
	dt = new dist_time;
	dt->dist = 3;
	dt->time = 7;
	dist_times.push_back(dt);
	
	TSND* tsnd = new TSND(NULL, 0);

	tsnd->compress(dist_times, comp_dist_times);

	for(std::list< dist_time* >::iterator it = comp_dist_times.begin();
		it != comp_dist_times.end(); ++it)
	{
		std::cout << (*it)->dist << " " << (*it)->time << std::endl;
	}

	Trajectory::delete_dist_times(comp_dist_times);
	Trajectory::delete_dist_times(dist_times);

	delete tsnd;

	return true;
}




















