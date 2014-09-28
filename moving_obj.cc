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
 *	FILE moving_obj.cc: Moving object processing
**/

/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <fstream>

#include "io.h"
#include "moving_obj.h"

const double Trajectory::max_speed = 1000; // m/s
const double Trajectory::min_speed = 0;

update* new_update(const std::string obj, const double latit, 
	const double longit, unsigned int time)
{
	update* up = new update;
	up->object = obj;
	up->latit = latit;
	up->longit = longit;
	up->time = time;

	return up;
}

seg_time* new_seg_time(const unsigned int segment, const unsigned int time)
{
	seg_time* st = new seg_time;
	st->segment = segment;
	st->time = time;

	return st;
}

Trajectory::Trajectory()
{
	size_traj = 0;
}
		
Trajectory::~Trajectory()
{
	for(std::list< seg_time* >::iterator it = seg_time_lst.begin();
		it != seg_time_lst.end(); ++it)
	{
		delete *it;
	}
}

void Trajectory::add_update(const unsigned int segment, const unsigned int timestamp)
{
	seg_time* st = new_seg_time(segment, timestamp);
	seg_time_lst.push_back(st);
	size_traj++;
}

const unsigned int Trajectory::build_trajectories(std::list<Trajectory*>& trajectories, 
	const std::list<update*>& updates, const RoadNet* net)
{
	double prev_latit;
	double prev_longit;
	unsigned int prev_time;
	double curr_latit;
	double curr_longit;
	unsigned int curr_time;
	unsigned int segment;
	std::vector<unsigned int> segments;
	std::vector<unsigned int> time_start_seg;
	double time_between_segs;
	double max_dist;
	
	segments.reserve(updates.size());
	time_start_seg.reserve(updates.size());
	
	prev_latit = updates.front()->latit;
	prev_longit = updates.front()->longit;
	prev_time = updates.front()->time;

	/*Assigning road segments for each update*/
	/*We are pushing the position to the start of the segment,
	 * therefore, the last update has no associated segment.*/
	std::list<update*>::const_iterator u = updates.begin();
	++u;
	
	for(; u != updates.end(); ++u)
	{
		curr_latit = (*u)->latit;
		curr_longit = (*u)->longit;
		curr_time = (*u)->time;
		
		if(segments.size())
		{
			time_between_segs = prev_time - time_start_seg.back();
			max_dist = (double) max_speed * time_between_segs;
			
			segment = net->match_segment(prev_latit, prev_longit, segments.back(), max_dist);
		}
		else
		{
			segment = net->match_segment(prev_latit, curr_latit, prev_longit, curr_longit);
		}

		if(! segments.size() || segment != segments.back())
		{
			segments.push_back(segment);
			time_start_seg.push_back(prev_time);
		}

//		std::cout << prev_latit << "," << prev_longit << "," << prev_time << " => " << segment << std::endl;

		prev_latit = curr_latit;
		prev_longit = curr_longit;
		prev_time = curr_time;
	}

	std::list<unsigned int> short_path;
	double dist_path;
	double speed;
	Trajectory* traj = new Trajectory();
	unsigned int tstmp;
	
//	std::cout << "size = " << segments.size() << std::endl;

	/*Completing trajectories using shortest paths*/
	for(unsigned int s = 1; s < segments.size(); s++)
	{
		traj->add_update(segments.at(s-1), time_start_seg.at(s-1));
		time_between_segs = time_start_seg.at(s) - time_start_seg.at(s-1);
		
		max_dist = (double) max_speed * time_between_segs;

		dist_path = net->shortest_path(short_path, segments.at(s-1), 
			segments.at(s), max_dist);

		speed = (double) dist_path / time_between_segs;
		
		if(speed < min_speed || speed > max_speed)
		{
			trajectories.push_back(traj);
			traj = new Trajectory();
		}
		else
		{
			dist_path = net->segment_length(segments.at(s-1));

			for(std::list<unsigned int>::iterator it = short_path.begin();
				it != short_path.end(); ++it)
			{
				/*Estimating timestamps assuming constant speed*/
				tstmp = time_start_seg.at(s-1) + 
					(unsigned int) ceil((double) dist_path / speed);
				traj->add_update(*it, tstmp);
				
				dist_path += net->segment_length(*it);
			}
		}
	}
	
	if(segments.size())
	{
		traj->add_update(segments.back(), time_start_seg.back());

		trajectories.push_back(traj);
	}
	
	return trajectories.size();
}

void delete_list_updates(std::list<update*>* updates)
{
	for(std::list<update*>::iterator it = updates->begin();
		it != updates->end(); ++it)
	{
		delete *it;
	}
}

std::list<Trajectory*>* Trajectory::read_trajectories(const std::string input_file_name,
	const RoadNet* net) throw (std::ios_base::failure)
{
	std::ifstream trajectories_file(input_file_name.c_str(), std::ios::in);
	std::string line_str;
	std::vector< std:: string > line_vec;
	 
	if(! trajectories_file.is_open())
	{
		std::cerr << "Error: Could not open trajectories file"
			<< input_file_name << std::endl << std::endl;
		
		return NULL;
	}

	std::map<std::string, unsigned int> object_ids;
	std::string id;
	std::getline(trajectories_file, line_str);
	unsigned int num_id = 0;
	
	while(! trajectories_file.eof())
	{
		line_vec = split(line_str,',');

		if(line_vec.size() < 5)
		{
			 std::cerr << "Error: Invalid trajectory file format, check the README file"
			 	<< input_file_name << std::endl << std::endl;
			 trajectories_file.close();

			 return NULL;
		}

		id = line_vec[0];
		
		if(object_ids.find(id) == object_ids.end())
		{
			object_ids[id] = num_id;
			num_id++;
		}

		std::getline(trajectories_file, line_str);
	}

	trajectories_file.clear();
	trajectories_file.seekg(0);
		
	std::list<Trajectory*>* trajectories = new std::list<Trajectory*>;
	unsigned int timestamp;
	double latit;
	double longit;
	std::vector<std::list< update* > * > updates;
	updates.reserve(num_id);

	for(unsigned int u = 0; u < num_id; u++)
	{
		updates.push_back(new std::list< update*>);
	}

	std::getline(trajectories_file, line_str);

	while(! trajectories_file.eof())
	{
		line_vec = split(line_str,',');

		if(line_vec.size() < 5)
		{
			 std::cerr << "Error: Invalid trajectory file format, check the README file"
			 	<< input_file_name << std::endl << std::endl;
			 trajectories_file.close();

			 return NULL;
		}

		id = line_vec[0];
		latit = atof(line_vec[1].c_str());
		longit = atof(line_vec[2].c_str());
		std::stringstream ss(line_vec[4]);
		ss >> timestamp;

		num_id = object_ids[id];

		updates.at(num_id)->push_back(new_update(id, latit, longit, timestamp));

		std::getline(trajectories_file, line_str);
	}

	trajectories_file.close();

	for(unsigned int u = 0; u < updates.size(); u++)
	{
		build_trajectories(*trajectories, *(updates.at(u)), net);
		
		delete_list_updates(updates.at(u));
		delete updates.at(u);
	}

	return trajectories;
}

void Trajectory::delete_trajectories(std::list<Trajectory*>* trajectories)
{
	for(std::list<Trajectory*>::iterator it = trajectories->begin();
		it != trajectories->end(); ++it)
	{
		delete *it;
	}
}

