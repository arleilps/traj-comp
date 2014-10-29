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
	prob = 0;
}

Trajectory::Trajectory(const Trajectory& traj)
{
	prob = traj.prob;

	for(std::list< seg_time* >::const_iterator it = traj.seg_time_lst.begin();
		it != traj.seg_time_lst.end(); ++it)
	{
		seg_time* st;
		st = *it;
		add_update(st->segment, st->time);
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

void Trajectory::add_update(const unsigned int segment, const unsigned int timestamp)
{
	seg_time* st = new_seg_time(segment, timestamp);
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
//	std::cout << "x_a = " << x_a << " y_a = " << y_a << " x_b = " << x_b << " y_b = " << y_b << " x = " << x << " y = " << y << std::endl;
	
	double a = pow(x_b-x_a, 2) + pow(y_b-y_a, 2);
	double b = 2.0 * ((x_a-x) * (x_b-x_a) + (y_a-y) * (y_b-y_a));
	double c = pow(x_a - x, 2) + pow(y_a-y, 2);

//	std::cout << "a = " << a << " b = " << b << " c = " << c << std::endl;

	double exp_val = 0;
	double cdf_param_1 = 0;
	double cdf_param_2 = 0;
	
	if(a != 0)
	{
		exp_val = (double) ( ((double) pow(b, 2) / (4.0 * a) - c) / (2.0 * pow(sigma, 2)));
		cdf_param_1 = (double) (2.0 * a + b) / (2.0 * sigma * sqrt(a));
		cdf_param_2 = (double) b / (2.0 * sigma * sqrt(a));
	}

//	std::cout << "exp_val = " << exp_val << " cdf_param_1 = " << cdf_param_1 << " cdf_param_2 = " << cdf_param_2 << std::endl;

//	std::cout << "exp = " << exp(exp_val) << " cdf_normal_1 = " << cdf_normal(sigma, cdf_param_1) << " cdf_normal_2 = " << cdf_normal(sigma, cdf_param_2) << std::endl;

	return exp(exp_val) * (cdf_normal(sigma, cdf_param_1) - cdf_normal(sigma, cdf_param_2));
}

std::vector < std::pair < unsigned int, double > * > 
	Trajectory::cand_seg_probs(const update* up, const RoadNet* net)
{
	double radius = 10 * sigma;
	std::list < unsigned int > cand_segs;
	unsigned int num_segs = net->segments_within_distance(cand_segs, up->latit, up->longit, radius);

	std::vector < std::pair <unsigned int, double > * > seg_probs;
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
		
//		std::cout << "prob = " << prob << std::endl;

		sum = sum + prob;
		seg_probs.push_back(new std::pair<unsigned int, double>(*it, prob));
	}

	for(unsigned int i = 0; i < seg_probs.size(); i++)
	{
		seg_probs.at(i)->second = (double) seg_probs.at(i)->second / sum;
	}

	return seg_probs;
}

const double Trajectory::transition_prob(const unsigned int seg_from, const unsigned int seg_to, 
	const double latit_from, const double latitt_to, 
	const double longit_from, const double longit_to, 
	const unsigned int time_from, const unsigned int time_to,
	const RoadNet* net)
{
	double gcd = net->distance_points(latit_from, latitt_to, longit_from, longit_to);
	double sd = net->shortest_path(seg_from, seg_to, std::numeric_limits<double>::max());

//	std::cout << "gcd = " << gcd << " sd = " << sd << std:: endl;

	double d = fabs(gcd-sd);
	double beta = beta_const + (double) (time_to - time_from) / time_div;
	
//	std::cout << "d = " << d << " beta = " << beta << " v = " << (double) exp(-(double) d / beta) / beta << std::endl;

	return (double) exp(-(double) d / beta) / beta;
}

const unsigned int Trajectory::build_trajectories(std::list<Trajectory*>& trajectories, 
	const std::list<update*>& updates, const RoadNet* net)
{
	//std::cout << seg_prob(2.873, 2.079, -9.578, -34.866, 0, 0, net, 7) << std::endl;
	//std::cout << seg_prob(-5.336, 28.854, -12.041, -24.578, 0, 0, net, 7) << std::endl;

	std::list<update*>::const_iterator it = updates.begin();
	update* up = *it;

	std::vector < std::pair < unsigned int, double > * > seg_probs;
	
	seg_probs = cand_seg_probs(up, net);
	
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

//	std::cout << "seg_probs.size() = " << seg_probs.size() << std::endl;

	for(unsigned int c = 0; c < seg_probs.size(); c++)
	{
		matrix_prob[0]->insert(std::pair<unsigned int, double>
			(seg_probs.at(c)->first, seg_probs.at(c)->second));
		traj = new Trajectory;
		traj->add_update(seg_probs.at(c)->first, up->time);
		matrix_traj[0]->insert(std::pair<unsigned int, Trajectory*>
			(seg_probs.at(c)->first, traj));
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

	for(; it != updates.end(); ++it)
	{
		std::cout << "u = " << u << std::endl;
		prev_up = up;
		up = *it;
		
		seg_probs = cand_seg_probs(up, net);
	//	std::cout << "seg_probs.size() = " << seg_probs.size() << std::endl;
		
		for(unsigned int s = 0; s < seg_probs.size(); s++)
		{
//			std::cout << "s = " << s << std::endl;
			seg = seg_probs.at(s)->first;
			max_prob = 0;
			max_seg = 0;

			for(it_traj = matrix_traj.at(u-1)->begin(); it_traj != matrix_traj.at(u-1)->end(); ++it_traj)
			{
				traj = it_traj->second;
				prev_seg = it_traj->first;
				
				trans_p = transition_prob(prev_seg, seg, prev_up->latit, up->latit, 
					prev_up->longit, up->longit, prev_up->time, up->time, net);
				
				prob = matrix_prob[u-1]->at(prev_seg) * trans_p * seg_probs.at(s)->second;

//				std::cout << "prev_seg = " << prev_seg << " prob = " << prob << " m = " <<  matrix_prob[u-1]->at(prev_seg) << std::endl;

				if(prob >= max_prob)
				{
					max_prob = prob;
					max_seg = prev_seg;
				}
			}

			matrix_prob[u]->insert(std::pair<unsigned int, double>
				(seg, max_prob));
			
//			std::cout << "max_seg = " << max_seg << std::endl;

			traj = new Trajectory(*(matrix_traj[u-1]->at(max_seg)));
			
			traj->add_update(seg, up->time);
			traj->prob = max_prob;
			
			matrix_traj[u]->insert(std::pair<unsigned int, Trajectory*>
				(seg, traj));
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
	
	Trajectory* most_likely_traj = NULL;
	max_prob = 0;

	for(it_traj = matrix_traj[u-1]->begin(); it_traj != matrix_traj[u-1]->end(); ++it_traj)
	{
		traj = it_traj->second;
		
		if(traj->prob > 
			max_prob)
		{
			max_prob = traj->prob;

			if(most_likely_traj != NULL)
			{
				delete most_likely_traj;
			}

			most_likely_traj = traj;
		}
		else
		{
			delete traj;
		}
	}

	trajectories.push_back(most_likely_traj);

	return 1;
}
/*
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

	for(unsigned int s = 1; s < segments.size(); s++)
	{
		traj->add_update(segments.at(s-1), time_start_seg.at(s-1));
		time_between_segs = time_start_seg.at(s) - time_start_seg.at(s-1);
		
		max_dist = (double) max_speed * time_between_segs;

		dist_path = net->shortest_path(short_path, segments.at(s-1), 
			segments.at(s), max_dist);

		speed = (double) dist_path / time_between_segs;
		
		std::cout << "speed: " << speed << std::endl;
		
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
*/

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
	double proj_latit;
	double proj_longit;
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
		
		net->project(latit, longit, proj_latit, proj_longit);
		updates.at(num_id)->push_back(new_update(id, latit, longit, proj_latit,
			proj_longit, timestamp));

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

