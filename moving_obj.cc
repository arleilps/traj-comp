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
#include <limits>

#include "io.h"
#include "moving_obj.h"

const double Trajectory::time_div = 10;
const double Trajectory::sigma = 7;
const double Trajectory::beta_const = 1;

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

update* new_update(const std::string obj, const double latit, 
	const double longit, const double proj_latit,
	const double proj_longit, unsigned int time)
{
	update* up = new update;
	up->object = obj;
	up->latit = latit;
	up->longit = longit;
	up->proj_latit = proj_latit;
	up->proj_longit = proj_longit;
	up->time = time;

	return up;
}

seg_time* new_seg_time(const unsigned int segment, const unsigned int time, const update* up)
{
	seg_time* st = new seg_time;
	st->segment = segment;
	st->time = time;
	st->up = up;

	return st;
}

Trajectory::Trajectory()
{
	size_traj = 0;
	prob = 0;
}

Trajectory::Trajectory(const Trajectory& traj)
{
	prob = traj.prob;
	size_traj = 0;
	for(std::list< seg_time* >::const_iterator it = traj.seg_time_lst.begin();
		it != traj.seg_time_lst.end(); ++it)
	{
		seg_time* st;
		st = *it;

		add_update(st->segment, st->time, st->up);
	}
}
		
Trajectory::~Trajectory()
{
	for(std::list< seg_time* >::iterator it = seg_time_lst.begin();
		it != seg_time_lst.end(); ++it)
	{
		delete *it;
	}
}

void Trajectory::add_update(const unsigned int segment, const unsigned int timestamp, const update* up)
{
	seg_time* st = new_seg_time(segment, timestamp, up);
	seg_time_lst.push_back(st);
	size_traj++;
}

const double cdf_normal(const double sigma, const double x)
{
	return (double) (1.0 + erf((double) x / sqrt(2))) / 2;
}

const double seg_prob(const double x_a, const double x_b, 
	const double y_a, const double y_b, 
	const double x, const double y, 
	const RoadNet* net, const double sigma)
{
	double a = pow(x_b-x_a, 2) + pow(y_b-y_a, 2);
	double b = 2.0 * ((x_a-x) * (x_b-x_a) + (y_a-y) * (y_b-y_a));
	double c = pow(x_a - x, 2) + pow(y_a-y, 2);

	double exp_val = 0;
	double cdf_param_1 = 0;
	double cdf_param_2 = 0;
	
	if(a != 0)
	{
		exp_val = (double) ( ((double) pow(b, 2) / (4.0 * a) - c) / (2.0 * pow(sigma, 2)));
		cdf_param_1 = (double) (2.0 * a + b) / (2.0 * sigma * sqrt(a));
		cdf_param_2 = (double) b / (2.0 * sigma * sqrt(a));
	}

	return (exp(exp_val) * 
		(cdf_normal(sigma, cdf_param_1) - cdf_normal(sigma, cdf_param_2))) 
		+ std::numeric_limits<double>::epsilon();
}

void Trajectory::cand_seg_probs(
		std::vector< std::pair < unsigned int, double > * >& seg_probs, 
		const update* up, const RoadNet* net)
{
	double radius = 10 * sigma;
	std::list < unsigned int > cand_segs;
	
	unsigned int num_segs = net->segments_within_distance(cand_segs, up->latit, up->longit, radius);
	seg_probs.clear();

	seg_probs.reserve(num_segs);
	double sum = 0;
	double prob;
	segment* seg;

	for(std::list<unsigned int>::iterator it = cand_segs.begin(); it != cand_segs.end(); ++it)
	{
		seg = net->get_segment(*it);
		prob = seg_prob(seg->proj_latit_begin, seg->proj_latit_end,
			seg->proj_longit_begin, seg->proj_longit_end, 
			up->proj_latit, up->proj_longit, net, sigma);
		
		sum = sum + prob;
		seg_probs.push_back(new std::pair<unsigned int, double>(*it, prob));
	}
	
	for(unsigned int i = 0; i < seg_probs.size(); i++)
	{
		seg_probs.at(i)->second = (double) seg_probs.at(i)->second / sum;
	}

	if(seg_probs.size() == 0)
	{
		unsigned int closest = net->closest_segment(up->latit, up->longit);
		seg_probs.push_back(new std::pair<unsigned int, double>(closest, 1));
	}
}

const double Trajectory::transition_prob(const unsigned int seg_from, const unsigned int seg_to, 
	const double latit_from, const double latit_to, 
	const double longit_from, const double longit_to, 
	const unsigned int time_from, const unsigned int time_to,
	const RoadNet* net)
{
	double gcd = net->distance_points(latit_from, latit_to, longit_from, longit_to);
	double sd = net->shortest_path(seg_from, seg_to, 
		latit_from, latit_to, longit_from, longit_to);

	double d = fabs(gcd-sd);
	double beta = beta_const + (double) (time_to - time_from) / time_div;

	return ((double) exp(-(double) d / beta) / beta) + std::numeric_limits<double>::epsilon();
}

Trajectory* Trajectory::map_matching(const std::list<update*>& updates, const RoadNet* net)
{
	std::list<update*>::const_iterator it = updates.begin();
	update* up = *it;

	std::vector < std::pair < unsigned int, double > * > seg_probs;
	
	cand_seg_probs(seg_probs, up, net);
	
	std::vector< std::map < unsigned int, double > * > matrix_prob;
	std::vector< std::map < unsigned int, Trajectory* > * > matrix_traj;
	matrix_prob.reserve(updates.size());
	matrix_traj.reserve(updates.size());
	
	for(unsigned int u = 0; u < updates.size(); u++)
	{
		matrix_prob.push_back(new std::map<unsigned int, double >);
		matrix_traj.push_back(new std::map<unsigned int, Trajectory* >);
	}

	Trajectory* traj;

	for(unsigned int c = 0; c < seg_probs.size(); c++)
	{
		matrix_prob[0]->insert(std::pair<unsigned int, double>
			(seg_probs.at(c)->first, -1 * log(seg_probs.at(c)->second)));
		traj = new Trajectory;
		traj->add_update(seg_probs.at(c)->first, up->time, up);
		traj->prob = -1 * log(seg_probs.at(c)->second);
		matrix_traj[0]->insert(std::pair<unsigned int, Trajectory*>
			(seg_probs.at(c)->first, traj));
		
		delete seg_probs.at(c);
	}

	double max_prob;
	unsigned int max_seg;
	double prob;
	unsigned int u = 1;
	double trans_p;
	update* prev_up;
	unsigned int seg;
	unsigned int prev_seg;
	std::map<unsigned int, Trajectory*>::iterator it_traj;
	++it;

	for(; it != updates.end(); ++it)
	{
		prev_up = up;
		up = *it;
		
		cand_seg_probs(seg_probs, up, net);
		
		for(unsigned int s = 0; s < seg_probs.size(); s++)
		{
			seg = seg_probs.at(s)->first;
			max_prob = std::numeric_limits<double>::max();
			max_seg = 0;

			for(it_traj = matrix_traj.at(u-1)->begin(); it_traj != matrix_traj.at(u-1)->end(); ++it_traj)
			{
				traj = it_traj->second;
				prev_seg = it_traj->first;
					
				trans_p = transition_prob(prev_seg, seg, prev_up->latit, up->latit, 
					prev_up->longit, up->longit, prev_up->time, up->time, net);
			
				try
				{
					prob = matrix_prob[u-1]->at(prev_seg) - log(trans_p)  - log(seg_probs.at(s)->second);
				}
				catch(const std::out_of_range& oor)
				{
					std::cout << "error 1" << std::endl;
					std::cout << "prev_seg = " << prev_seg << std::endl;
					std::cout << "m = " << (matrix_prob[u-1]->begin())->first << std::endl;
					std::cout << matrix_prob[u-1]->size() << std::endl;
					exit(1);
				}

				if(prob <= max_prob)
				{
					max_prob = prob;
					max_seg = prev_seg;
				}
			}

			matrix_prob[u]->insert(std::pair<unsigned int, double>
				(seg, max_prob));

			try
			{
				traj = new Trajectory(*(matrix_traj[u-1]->at(max_seg)));
			}
			catch(const std::out_of_range& oor)
			{
				std::cout << "error 2" << std::endl;
				std::cout << matrix_traj[u-1]->size() << std::endl;
				std::cout << "max_seg = " << max_seg << std::endl;
				std::cout << "m = " << (matrix_traj[u-1]->begin())->first << std::endl;
				exit(1);
			}

			traj->add_update(seg, up->time, up);
			traj->prob = max_prob;
			
			matrix_traj[u]->insert(std::pair<unsigned int, Trajectory*>
				(seg, traj));

			delete seg_probs.at(s);
		}
		
		for(it_traj = matrix_traj.at(u-1)->begin(); it_traj != matrix_traj.at(u-1)->end(); ++it_traj)
		{
			traj = it_traj->second;
			delete traj;
		}
		
		delete matrix_prob[u-1];
		delete matrix_traj[u-1];

		u++;
	}

	max_prob = std::numeric_limits<double>::max();
	Trajectory* most_likely_traj = NULL;

	for(it_traj = matrix_traj[u-1]->begin(); it_traj != matrix_traj[u-1]->end(); ++it_traj)
	{
		traj = it_traj->second;
		
		if(traj->prob <= 
			max_prob)
		{
			max_prob = traj->prob;
			most_likely_traj = traj;
		}
	}
	
	Trajectory* res;
	
	if(most_likely_traj != NULL)
	{
		res = new Trajectory(*most_likely_traj);
	}
	else
	{
		//TODO: Add exception
		res = NULL;
		std::cout << "Map matching error, most likely trajectory has 0 probability" << std::endl;
	}
	
	for(it_traj = matrix_traj[u-1]->begin(); it_traj != matrix_traj[u-1]->end(); ++it_traj)
	{
		traj = it_traj->second;
		delete traj;
	}

	delete matrix_traj[u-1];
	delete matrix_prob[u-1];
	
	return res;
}

void Trajectory::extend_traj_shortest_paths(const RoadNet*net)
{
	std::list<unsigned int> sp;
	std::list< seg_time* >::iterator st_next;
	
	for(std::list< seg_time* >::iterator st = seg_time_lst.begin();
		st != seg_time_lst.end(); ++st)
	{
		st_next = st;
		st_next++;
		
		if(st_next != seg_time_lst.end())
		{
			net->shortest_path(sp, (*st)->segment, (*st_next)->segment, 
				(*st)->up->latit, (*st_next)->up->latit, 
				(*st)->up->longit, (*st_next)->up->longit);
			
			for(std::list<unsigned int>::iterator s = sp.begin(); s != sp.end(); ++s)
			{
				seg_time_lst.insert(st_next, new_seg_time(*s, 0));
				++st;
			}

			sp.clear();
		}
		else
		{
			break;
		}
	}
}

void Trajectory::remove_repeated_segments()
{
	std::list< seg_time* >::iterator st_next;
	
	std::list< seg_time* >::iterator st = seg_time_lst.begin();

	while(st != seg_time_lst.end())
	{
		st_next = st;
		st_next++;

		if(st_next != seg_time_lst.end())
		{
			if((*st)->segment == (*st_next)->segment)
			{
				st = seg_time_lst.erase(st);
			}
			else
			{
				++st;
			}
		}
		else
		{
			break;
		}
	}
}

const unsigned int Trajectory::read_trajectories(std::list< Trajectory * >& trajectories,
	const std::string input_file_name, 
	const RoadNet* net) throw (std::ios_base::failure)
{
	return 0;
}

const unsigned int Trajectory::read_updates(std::vector<std::list< update* > * >& updates,
	const std::string input_file_name,
	const RoadNet* net) 
	throw (std::ios_base::failure)
{
	std::ifstream trajectories_file(input_file_name.c_str(), std::ios::in);
	std::string line_str;
	std::vector< std:: string > line_vec;
	 
	if(! trajectories_file.is_open())
	{
		std::cerr << "Error: Could not open trajectories file"
			<< input_file_name << std::endl << std::endl;
		
		return 0;
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

			 return 0;
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
		
	unsigned int timestamp;
	double latit;
	double longit;
	double proj_latit;
	double proj_longit;
	updates.clear();
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

			 return 0;
		}

		id = line_vec[0];
		latit = atof(line_vec[1].c_str());
		longit = atof(line_vec[2].c_str());
		std::stringstream ss(line_vec[4]);
		ss >> timestamp;

		num_id = object_ids[id];
		
		net->project(latit, longit, proj_latit, proj_longit);
		updates.at(num_id)->push_back(new_update(id, latit, longit, proj_latit,
			proj_longit, timestamp));

		std::getline(trajectories_file, line_str);
	}

	trajectories_file.close();

	return updates.size();
}

void Trajectory::delete_trajectories(std::list<Trajectory*>* trajectories)
{
	for(std::list<Trajectory*>::iterator it = trajectories->begin();
		it != trajectories->end(); ++it)
	{
		delete *it;
	}
}

void Trajectory::print() const
{
	for(std::list< seg_time* >::const_iterator it = seg_time_lst.begin();
		it != seg_time_lst.end(); ++it)
	{
		std::cout << "(" << (*it)->segment << " , " << (*it)->time << ") ";
	}

	std::cout << std::endl;
}

void delete_list_updates(std::list<update*>* updates)
{
	for(std::list<update*>::iterator it = updates->begin(); 
		it != updates->end(); ++it)
	{
		delete *it;
	}
}
