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
	std::cout << "Testing moving object functionalities" << std::endl;

	/*Creating a road network*/
	std::cout << "Creating a road network" << std::endl;
	RoadNet* net = new RoadNet("../data/greater_sfo_adj.txt");
	std::cout << "Road network created" << std::endl;

	/*Generating trajectories from updates*/
	std::cout << "Generating trajectories from updates" << std::endl;
	
	std::string object;
	double latit;
	double longit;
	unsigned int timestamp;
	std::list<update*> updates;
	
	object = "aldhidd";

//	aldhidd 37.75108 -122.39488 0 1213086585
	latit = 37.75108;
	longit = -122.39488;
	timestamp = 1213086585;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75124 -122.39492 0 1213086636
	latit = 37.75124;
	longit = -122.39492;
	timestamp = 1213086636;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75125 -122.39493 0 1213086696
	latit = 37.7512;
	longit = -122.39493;
	timestamp = 1213086696;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75121 -122.39492 0 1213086757
	latit = 37.75121;
	longit = -122.39492;
	timestamp = 1213086757;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75121 -122.39493 0 1213086817
	latit = 37.75121;
	longit = -122.39493;
	timestamp = 1213086817;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75121 -122.39494 0 1213086877
	latit = 37.75121;
	longit = -122.39494;
	timestamp = 1213086877;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75122 -122.39495 0 1213086937
	latit = 37.75122;
	longit = -122.39495;
	timestamp = 1213086937;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.7512 -122.39494 0 1213086998
	latit = 37.7512;
	longit = -122.39494;
	timestamp = 1213086998;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75122 -122.39508 0 1213087057
	latit = 37.75122;
	longit = -122.39508;
	timestamp = 1213087057;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75122 -122.39498 0 1213087118
	latit = 37.75122;
	longit = -122.39498;
	timestamp = 1213087118;
	
	updates.push_back(new_update(object, latit, longit, timestamp));
	
	std::cout << "updates:" << std::endl;
	for(std::list<update*>::iterator u = updates.begin();
		u != updates.end(); ++u)
	{
		std::cout  << " <" << (*u)->latit <<
			" , " << (*u)->longit << 
			" , " << (*u)->time << ">" << std::endl;
	}

	std::list<Trajectory*> trajectories;

	unsigned int count;
	unsigned int t;
	count = Trajectory::build_trajectories(trajectories, updates, net);
	t = 0;
	std::cout << count << " trajectories:" << std::endl;

	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it) 
	{
		std::cout << "Trajectory: " << t << " size = " << (*it)->size() << std::endl;
		
		for(Trajectory::iterator traj_it = (*it)->begin(); traj_it != (*it)->end(); ++traj_it)
		{
			std::cout << "<seg: " << (*traj_it)->segment <<
				" time: " << (*traj_it)->time << ">" << std::endl;
		}
		
		std::cout << std::endl;

		delete *it;
		t++;
	}
	
	trajectories.clear();
	
	/*Simulating case when timestamps are too far away*/
	std::cout << "Simulating case when object is too slow" << std::endl;
	
//	aldhidd 37.75123 -122.39496 0 1213088400
	latit = 37.75123;
	longit = -122.39496;
	timestamp = 1213098400;

	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75118 -122.39492 0 1213088600
	latit = 37.75118;
	longit = -122.39492;
	timestamp = 1213098600;

	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75119 -122.39492 0 1213088800
	latit = 37.75118;
	longit = -122.39492;
	timestamp = 1213098800;

	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75131 -122.39504 0 1213089000
	latit = 37.75131;
	longit = -122.39504;
	timestamp = 1213099000;

	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75132 -122.39501 0 1213089200
	latit = 37.75132;
	longit = -122.39501;
	timestamp = 1213099200;

	updates.push_back(new_update(object, latit, longit, timestamp));

//	aldhidd 37.75131 -122.39497 0 1213089400
	latit = 37.75131;
	longit = -122.39497;
	timestamp = 1213099400;

	updates.push_back(new_update(object, latit, longit, timestamp));
	
	std::cout << "updates:" << std::endl;
	for(std::list<update*>::iterator u = updates.begin();
		u != updates.end(); ++u)
	{
		std::cout  << " <" << (*u)->latit <<
			" , " << (*u)->longit << 
			" , " << (*u)->time << ">" << std::endl;
	}

	count = Trajectory::build_trajectories(trajectories, updates, net);
	t = 0;
	std::cout << count << " trajectories:" << std::endl;

	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it) 
	{
		std::cout << "Trajectory: " << t << " size = " << (*it)->size() << std::endl;
		
		for(Trajectory::iterator traj_it = (*it)->begin(); traj_it != (*it)->end(); ++traj_it)
		{
			std::cout << "<seg: " << (*traj_it)->segment <<
				" time: " << (*traj_it)->time << ">" << std::endl;
		}
		
		std::cout << std::endl;

		delete *it;
		t++;
	}
	
	trajectories.clear();

	/*Simulating case when segments are too far away*/
	std::cout << "Simulating case when object is too fast" << std::endl;
	
//	aydwaho 37.75118 -122.39343 0 1213089600
	latit = 37.75118;
	longit = -122.39343;
	timestamp = 1213099400;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aydwaho 37.75146 -122.39525 0 1213089800
	latit = 37.75146;
	longit = -122.39525;
	timestamp = 1213099401;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aydwaho 37.75141 -122.39516 0 1213081000
	latit = 37.75141;
	longit = -122.39516;
	timestamp = 1213099402;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aydwaho 37.75144 -122.39512 0 1213081200
	latit = 37.75144;
	longit = -122.39512;
	timestamp = 1213099403;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aydwaho 37.75138 -122.39513 0 1213081400
	latit = 37.75138;
	longit = -122.39513;
	timestamp = 1213099404;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aydwaho 37.7517 -122.39421 0 1213081600
	latit = 37.7517;
	longit = -122.39421;
	timestamp = 1213091405;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aydwaho 37.75149 -122.39425 0 1213081800
	latit = 37.75149;
	longit = -122.39425;
	timestamp = 1213091406;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aydwaho 37.7515 -122.39425 0 1213082000
	latit = 37.7515;
	longit = -122.39425;
	timestamp = 1213091407;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aydwaho 37.75151 -122.39425 0 1213082200
	latit = 37.75151;
	longit = -122.39425;
	timestamp = 1213091408;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aydwaho 37.75151 -122.39426 0 1213082400
	latit = 37.75151;
	longit = -122.39426;
	timestamp = 1213091409;
	
	updates.push_back(new_update(object, latit, longit, timestamp));

//	aydwaho 37.7515 -122.39427 0 1213082600
	latit = 37.7515;
	longit = -122.39427;
	timestamp = 1213091410;
	
	updates.push_back(new_update(object, latit, longit, timestamp));
	
	std::cout << "updates:" << std::endl;
	for(std::list<update*>::iterator u = updates.begin();
		u != updates.end(); ++u)
	{
		std::cout  << " <" << (*u)->latit <<
			" , " << (*u)->longit << 
			" , " << (*u)->time << ">" << std::endl;
	}

	count = Trajectory::build_trajectories(trajectories, updates, net);
	t = 0;
	std::cout << count << " trajectories:" << std::endl;

	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it) 
	{
		std::cout << "Trajectory: " << t << " size = " << (*it)->size() << std::endl;
		
		for(Trajectory::iterator traj_it = (*it)->begin(); traj_it != (*it)->end(); ++traj_it)
		{
			std::cout << "<seg: " << (*traj_it)->segment <<
				" time: " << (*traj_it)->time << ">" << std::endl;
		}
		
		std::cout << std::endl;

		delete *it;
		t++;
	}
	
	trajectories.clear();

	/*Simulating case when segments are too far away*/

	for(std::list<update*>::iterator u = updates.begin();
		u != updates.end(); ++u)
	{
		delete *u;
	}
	
	std::cout << "Deleting road network" << std::endl;
	delete net;
	std::cout << "Road network deleted" << std::endl;

	std::cout << "Test finished" << std::endl;

	return true;
}

const bool test_moving_obj_file()
{
	/*Creating a road network*/
	std::cout << "Creating a road network" << std::endl;
	RoadNet* net = new RoadNet("../data/greater_sfo_adj.txt");
	std::cout << "Road network created" << std::endl;

	std::cout << "Reading trajectories from file" << std::endl;
	std::list<Trajectory*>* trajectories = Trajectory::read_trajectories("../data/small_cab_stream_sfo.txt", net);
	std::cout << "Reading trajectories from file" << std::endl;

	std::cout << "Printing trajectories:" << std::endl;
	
	Trajectory* trajectory;

	for(std::list<Trajectory*>::iterator it = trajectories->begin(); 
		it != trajectories->end(); ++it)
	{
		trajectory = *it;

		for(Trajectory::iterator traj_it = trajectory->begin(); traj_it != trajectory->end(); ++traj_it)
		{
			std::cout << "<" << (*traj_it)->segment <<
				"," << (*traj_it)->time << "> ";
		}
		
		std::cout << std::endl;
	}
	
	std::cout << "Finished printing trajectories" << std::endl;
	
	std::cout << "Deleting trajectories" << std::endl;
	
	Trajectory::delete_trajectories(trajectories);
	
	std::cout << "Finished deleting trajectories" << std::endl;

	std::cout << "Deleting road network";
	delete net;
	std::cout << "Road network deleted";

	std::cout << "Test finished" << std::endl;

	return true;
}
	
