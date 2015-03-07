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
#include <pthread.h>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>

#include "io.h"
#include "moving_obj.h"

//Default values for constants used in map-matching
const double Trajectory::TIMEDIV = 10;
const double Trajectory::SIGMA = 4.07;
const double Trajectory::BETACONST = 1;
const double Trajectory::RADIUS = 30;
const double Trajectory::MAXSPEED = 32;	//in m/s
const unsigned int Trajectory::MAXCANDMATCHES = 5;
const double Trajectory::MAXLENGTHSHORTESTPATH = 2000;

double Trajectory::time_div = TIMEDIV;
double Trajectory::sigma = SIGMA;
double Trajectory::beta_const = BETACONST;
double Trajectory::radius = RADIUS;
unsigned int Trajectory::max_cand_matches = MAXCANDMATCHES;
unsigned int Trajectory::num_threads = 1;

/*
const std::string TrajDBPostGis::database_name = "test";
const std::string TrajDBPostGis::table_name = "traj";
const std::string TrajDBPostGis::host = "127.0.0.1";
const std::string TrajDBPostGis::port = "5432";
const std::string TrajDBPostGis::user = "traj_comp";
const std::string TrajDBPostGis::password = "traj_comp";

const std::string TrajDBPostGis::srid = "26943";
const std::string TrajDBPostGis::spatial_ref = "4326";
*/

update* new_update
	(
		const std::string obj, 
		const double latit, 
		const double longit, 
		unsigned int time
	)
{
	update* up = new update;
	up->object = obj;
	up->latit = latit;
	up->longit = longit;
	up->time = time;

	return up;
}

update* new_update
	(
		const std::string obj, 
		const double latit, 
		const double longit, 
		const double proj_latit,
		const double proj_longit, 
		unsigned int time
	)
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

seg_time* new_seg_time
	(
		const unsigned int segment, 
		const unsigned int time, 
		const double dist, 
		const update* up
	)
{
	seg_time* st = new seg_time;
	st->segment = segment;
	st->time = time;
	st->dist = dist;
	st->up = up;

	return st;
}

p_thread_param_map* new_p_thread_param_map
	(
		std::vector<std::list< update* > * >* updates,
		unsigned int* pointer,
		pthread_mutex_t* mutex_pool,
		pthread_mutex_t* mutex_file,
		std::ofstream* output_file,
		RoadNet* net
	)
{
	p_thread_param_map* param = new p_thread_param_map;
	
	param->updates = updates;
	param->pointer = pointer;
	param->mutex_pool = mutex_pool;
	param->mutex_file = mutex_file;
	param->output_file = output_file;
	param->net = net;

	return param;
}

p_thread_param_exp* new_p_thread_param_exp
	(
		std::list< Trajectory* >*  trajectories,
		std::list< Trajectory* >::iterator* pointer,
		pthread_mutex_t* mutex_pool,
		RoadNet* net
	)
{
	p_thread_param_exp* param = new p_thread_param_exp;
	
	param->trajectories = trajectories;
	param->pointer = pointer;
	param->mutex_pool = mutex_pool;
	param->net = net;

	return param;
}

dist_time* new_dist_time(const double dist, const unsigned int time)
{
	dist_time* dt = new dist_time;

	dt->time = time;
	dt->dist = dist;

	return dt;
}

Trajectory::Trajectory(const std::string _obj)
{
	size_traj = 0;
	prob = 0;
	obj = _obj;
}

Trajectory::Trajectory(const Trajectory& traj)
{
	prob = traj.prob;
	size_traj = 0;
	obj = traj.obj;
	for(std::list< seg_time* >::const_iterator it = traj.seg_time_lst.begin();
		it != traj.seg_time_lst.end(); ++it)
	{
		seg_time* st;
		st = *it;

		add_update(st->segment, st->time, st->dist, st->up);
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

void Trajectory::get_dist_times_uniform
	(
		std::list < dist_time* >& dist_times,
		RoadNet* net
	) 
		const
{
	std::list< seg_time* >::const_iterator iti = seg_time_lst.begin();
	std::list< seg_time* >::const_iterator itj;
	seg_time* sti;
	seg_time* stj;
	double dist;
	double total_dist;
	unsigned int s_time;
	unsigned int e_time;
	double speed;

	sti = (*iti);
	dist_times.push_back(new_dist_time(sti->dist, 0));
	total_dist = sti->dist;

	while(iti != seg_time_lst.end())
	{
		sti = (*iti);
		
		if(sti->time != 0 || sti->dist != 0)	//update, i.e. not shortest path completion
		{
			dist = net->segment_length(sti->segment) - sti->dist;

			itj = iti;
			++itj;

			if(itj != seg_time_lst.end())
			{
				while(itj != seg_time_lst.end() 
					&& (*itj)->time == 0 && (*itj)->dist == 0) //shortest-path extension
				{
					stj = (*itj);
					dist += net->segment_length(stj->segment);
					++itj;
				}

				stj = (*itj);

				if(stj->segment != sti->segment)
				{
					dist += stj->dist;
				}
				else
				{
					dist = stj->dist - sti->dist;
				}

				s_time = sti->time;
				e_time = stj->time;
				speed = (double) dist / (e_time-s_time);

				itj = iti;
				++itj;
				
				if(sti->segment != stj->segment)
				{
					dist = net->segment_length(sti->segment) - sti->dist;
				}

				e_time = s_time + dist / speed;
	
				while(itj != seg_time_lst.end() 
					&& (*itj)->time == 0 && (*itj)->dist == 0) //shortest-path extension
				{
					stj = (*itj);
					dist += net->segment_length(stj->segment);
					e_time = s_time + dist / speed;
					dist_times.push_back(new_dist_time(total_dist + dist, 
						e_time - seg_time_lst.front()->time));
					++itj;
				}
		
				stj = (*itj);
				
				if(sti->segment != stj->segment)
				{
					dist += stj->dist;
				}
				else
				{
					dist = stj->dist - sti->dist;
				}
				
				dist_times.push_back(new_dist_time(total_dist + dist, 
					stj->time - seg_time_lst.front()->time));
				total_dist += dist;
			}
			
			iti = itj;
		}
		else
		{
			++iti;
		}
	}
}

void Trajectory::get_dist_times_least_squares
	(
		std::list < dist_time* >& dist_times,
		const Eigen::VectorXd& f,
		RoadNet* net
	) 
		const
{
	std::list< seg_time* >::const_iterator iti = seg_time_lst.begin();
	std::list< seg_time* >::const_iterator itj;
	seg_time* sti;
	seg_time* stj;
	double dist;
	double total_dist;
	unsigned int s_time;
	unsigned int e_time;
	double conv_ratio;
	double pred_time;

	sti = (*iti);
	dist_times.push_back(new_dist_time(sti->dist, 0));
	total_dist = sti->dist;

	while(iti != seg_time_lst.end())
	{
		sti = (*iti);

		if(sti->time != 0 || sti->dist != 0)	//update, i.e. not shortest path completion
		{
			itj = iti;
			++itj;

			if(itj != seg_time_lst.end())
			{
				dist = net->segment_length(sti->segment) - sti->dist;
				pred_time = (double) (f[sti->segment] 
					* dist) / (net->segment_length(sti->segment));
				
				while(itj != seg_time_lst.end() 
					&& (*itj)->time == 0 && (*itj)->dist == 0) //shortest-path extension
				{
					stj = (*itj);
					pred_time += f[stj->segment];
					++itj;
				}

				stj = (*itj);
				
				if(sti->segment != stj->segment)
				{
					pred_time += f[stj->segment] 
						* (double) stj->dist / (net->segment_length(stj->segment));
				}
				else
				{
					dist = stj->dist - sti->dist;
					pred_time = (double) (f[sti->segment] 
					* dist) / (net->segment_length(sti->segment));
				}
			
				s_time = sti->time;
				e_time = stj->time;
				conv_ratio = (double) (e_time-s_time) / pred_time;
				itj = iti;
				++itj;
			
				dist = net->segment_length(sti->segment) - sti->dist;
				pred_time = (double) (f[sti->segment] 
					* dist) / (net->segment_length(sti->segment));
				e_time = s_time + pred_time * conv_ratio;

				while(itj != seg_time_lst.end() 
					&& (*itj)->time == 0 && (*itj)->dist == 0) //shortest-path extension
				{
					stj = (*itj);
					dist += net->segment_length(stj->segment);
					pred_time += f[stj->segment];
					e_time = s_time + pred_time * conv_ratio;
					dist_times.push_back(new_dist_time(total_dist + dist, 
						e_time - seg_time_lst.front()->time));
					++itj;
				}
			
				stj = (*itj);

				if(sti->segment != stj->segment)
				{
					dist += stj->dist;
					dist_times.push_back(new_dist_time(total_dist + dist, 
						stj->time - seg_time_lst.front()->time));
				}
				else
				{
					dist = stj->dist - sti->dist;
					dist_times.push_back(new_dist_time(total_dist + dist, 
						stj->time - seg_time_lst.front()->time));
				}
				
				total_dist += dist;
			}
			
			iti = itj;
		}
		else
		{
			++iti;
		}
	}
}

void Trajectory::get_pred_dist_times_least_squares
	(
		std::list < dist_time* >& pred_dist_times,
		const Eigen::VectorXd& f,
		RoadNet* net
	) 
		const
{
	std::list< seg_time* >::const_iterator iti = seg_time_lst.begin();
	seg_time* sti;
	seg_time* stj;
	double total_dist;
	unsigned int e_time = 0;
	double pred_time;

	sti = (*iti);
	pred_dist_times.push_back(new_dist_time(sti->dist, 0));
	total_dist = sti->dist;

	stj = sti;
	iti++;

	while(iti != seg_time_lst.end())
	{
		sti = (*iti);
		
		if(stj->segment != sti->segment)
		{
			if(sti->time != 0 || sti->dist != 0)
			{
				total_dist += sti->dist;
				pred_time = (double) (f[sti->segment] 
					* sti->dist) / (net->segment_length(sti->segment));
			}
			else
			{
				total_dist += net->segment_length(sti->segment);
				pred_time = f[sti->segment];
			}
			
			e_time += pred_time;
			
			if(stj->time != 0 || stj->dist != 0)
			{
				total_dist += net->segment_length(stj->segment) - stj->dist;
				pred_time = (double) (f[stj->segment] 
					* (net->segment_length(stj->segment) - stj->dist)) 
					/ (net->segment_length(stj->segment));
				e_time += pred_time;
			}

			pred_dist_times.push_back(new_dist_time(total_dist, e_time));
		}
		else
		{
			if(sti->time != 0 || sti->dist != 0)
			{
				total_dist += sti->dist - stj->dist;
			
				pred_time = (double) (f[sti->segment] 
					* (sti->dist - stj->dist)) 
					/ (net->segment_length(sti->segment));
			
				e_time += pred_time;
			}
			
			pred_dist_times.push_back(new_dist_time(total_dist, e_time));
		}

		stj = sti;

		++iti;
	}
}

void Trajectory::get_lsq_sigmas
	(
		std::vector<double>& lsq_sigmas,
		std::vector<unsigned int>& counts,
		const Eigen::VectorXd& f,
		RoadNet* net
	) 
		const
{
	std::list< seg_time* >::const_iterator iti = seg_time_lst.begin();
	std::list< seg_time* >::const_iterator itj;
	seg_time* sti;
	seg_time* stj;
	double dist;
	double diff;
	unsigned int s_time;
	unsigned int e_time;
	double conv_ratio;
	double pred_time;

	sti = (*iti);

	while(iti != seg_time_lst.end())
	{
		sti = (*iti);

		if(sti->time != 0 || sti->dist != 0)	//update, i.e. not shortest path completion
		{
			itj = iti;
			++itj;

			if(itj != seg_time_lst.end())
			{
				dist = net->segment_length(sti->segment) - sti->dist;
				pred_time = (double) (f[sti->segment] 
					* dist) / (net->segment_length(sti->segment));
				
				while(itj != seg_time_lst.end() 
					&& (*itj)->time == 0 && (*itj)->dist == 0) //shortest-path extension
				{
					stj = (*itj);
					pred_time += f[stj->segment];
					++itj;
				}

				stj = (*itj);
				
				if(sti->segment != stj->segment)
				{
					pred_time += f[stj->segment] 
						* (double) stj->dist / (net->segment_length(stj->segment));
				}
				else
				{
					dist = stj->dist - sti->dist;
					pred_time = (double) (f[sti->segment] 
					* dist) / (net->segment_length(sti->segment));
				}
			
				s_time = sti->time;
				e_time = stj->time;
				conv_ratio = (double) (e_time-s_time) / pred_time;
				itj = iti;
				++itj;
			
				diff = (conv_ratio - 1) * f[sti->segment];
				counts[sti->segment]++;
				lsq_sigmas[sti->segment] += pow(diff, 2);

				while(itj != seg_time_lst.end() 
					&& (*itj)->time == 0 && (*itj)->dist == 0) //shortest-path extension
				{
					stj = (*itj);
					diff = (conv_ratio - 1) * f[stj->segment];
					counts[stj->segment]++;
					lsq_sigmas[stj->segment] += pow(diff, 2);
					
					++itj;
				}
			
				stj = (*itj);
				
				if(stj->segment != sti->segment)
				{
					diff = (conv_ratio - 1) * f[stj->segment];
					counts[stj->segment]++;
					lsq_sigmas[stj->segment] += pow(diff, 2);
				}
			}
			
			iti = itj;
		}
		else
		{
			++iti;
		}
	}
}

void Trajectory::get_sparse_rep
	(
		std::vector < Eigen::Triplet<double> >& Q,
		std::vector < double >& y, 
		unsigned int& sz,
		RoadNet* net
	) 
		const
{
	seg_time* st;
	double frac;
	unsigned int time;
	unsigned int start;
	
	std::list < seg_time* >::const_iterator it_st = seg_time_lst.begin();
	st = *it_st;
	frac = (double) (net->segment_length(st->segment) - st->dist) / net->segment_length(st->segment);
	Q.push_back(Eigen::Triplet<double>(st->segment, sz, frac));
	start = st->time;
	++it_st;

	while(it_st != seg_time_lst.end())
	{
		st = *it_st;

		if(st->time == 0 && st->dist == 0)	//shortest path completion
		{
			frac = 1;
			Q.push_back(Eigen::Triplet<double>(st->segment, sz, frac));
		}
		else
		{
			frac = (double) st->dist / net->segment_length(st->segment);
			Q.push_back(Eigen::Triplet<double>(st->segment, sz, frac));
			time = st->time - start;
			y.push_back(time);
			sz++;
		}
		
		++it_st;

		if((st->time != 0 || st->dist != 0) &&
			it_st != seg_time_lst.end())
		{
			frac = (double) (net->segment_length(st->segment) - st->dist) 
				/ net->segment_length(st->segment);
			Q.push_back(Eigen::Triplet<double>(st->segment, sz, frac));
			start = st->time;
		}
	}
}

void Trajectory::delete_dist_times(std::list < dist_time* >& dist_times)
{
	for(std::list< dist_time* >::iterator it = dist_times.begin(); 
		it != dist_times.end(); ++it)
	{
		delete *it;
	}

	dist_times.clear();
}

void Trajectory::add_update
	(
		const unsigned int segment, 
		const unsigned int time, 
		const double dist, 
		const update* up
	)
{
	seg_time* st = new_seg_time(segment, time, dist, up);
	seg_time_lst.push_back(st);
	size_traj++;

	if(up != NULL)
	{
		obj = up->object;
	}
}

void Trajectory::add_update_front
	(
		const unsigned int segment, 
		const unsigned int time, 
		const double dist, 
		const update* up
	)
{
	seg_time* st = new_seg_time(segment, time, dist, up);
	seg_time_lst.push_front(st);
	size_traj++;

	if(up != NULL)
	{
		obj = up->object;
	}
}

const double cdf_normal(const double sigma, const double x)
{
	return (double) (1.0 + erf((double) x / sqrt(2))) / 2;
}

const double seg_prob
	(
		const double x_a, 
		const double x_b, 
		const double y_a, 
		const double y_b, 
		const double x, 
		const double y, 
		const RoadNet* net, 
		const double sigma
	)
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

struct cmp
{
	bool operator()(const std::pair<unsigned int, double>* p1,
		const std::pair<unsigned int, double>* p2) const
	{
 		return p1->second > p2->second;
	}
}compare_pair_segs;

void Trajectory::cand_seg_probs
	(
		std::vector< std::pair < unsigned int, double > * >& seg_probs, 
		const update* up, const RoadNet* net
	)
{
	double search_radius = radius * sigma;
	std::list < unsigned int > cand_segs;
	
	//Getting segments within a radius from the update
	unsigned int num_segs = net->segments_within_distance(cand_segs, 
		up->latit, up->longit, search_radius);
	seg_probs.clear();
	seg_probs.reserve(num_segs);
		
	double sum = 0;
	double prob;
	segment* seg;
	
	//Computing probabilies for each segment based on its distance
	for(std::list<unsigned int>::iterator it = cand_segs.begin(); 
		it != cand_segs.end(); ++it)
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

	//In case no segment is found, take the closest one
	if(seg_probs.size() == 0)
	{
		unsigned int closest = net->closest_segment(up->latit, up->longit);
		seg_probs.push_back(new std::pair<unsigned int, double>(closest, 1));
	}

	std::sort(seg_probs.begin(), seg_probs.end(), compare_pair_segs);

	while(seg_probs.size() > max_cand_matches)
	{
		delete seg_probs.back();
		seg_probs.pop_back();
	}
}

const double Trajectory::transition_prob
	(
		const unsigned int seg_from, 
		const unsigned int seg_to, 
		const double latit_from, 
		const double latit_to, 
		const double longit_from, 
		const double longit_to, 
		const unsigned int time_from, 
		const unsigned int time_to,
		const double gcd,
		const RoadNet* net
	)
{
	//Shortest path distance in the road network
	double time = time_to - time_from;
	double max_distance = MAXSPEED * time;
	double sd = net->shortest_path(seg_from, seg_to, 
		latit_from, latit_to, longit_from, longit_to, max_distance);
	
	//Probability grows as these two distances get closer
	double d = fabs(gcd-sd);
	double beta = beta_const + (double) (time_to - time_from) / time_div;
	
	//Adding small number so that probabilities are never 0.
	return ((double) exp(-(double) d / beta) / beta) 
		+ std::numeric_limits<double>::epsilon();
}

// In order to understand this code, please check:
// Map-matching algorithm: http://www.mit.edu/~jaillet/general/hmm-based-proximity.pdf
// Hidden markov model: http://en.wikipedia.org/wiki/Hidden_Markov_model
// Viterbi algorithm: http://en.wikipedia.org/wiki/Viterbi_algorithm
Trajectory* Trajectory::map_matching
	(
		const std::list<update*>& updates, 
		RoadNet* net
	)
{
	std::list<update*>::const_iterator it = updates.begin();
	update* up = *it;
	std::vector < std::pair < unsigned int, double > * > seg_probs;

	
	//Identifying candidate road segments for update
	cand_seg_probs(seg_probs, up, net);
	
	std::vector < Trajectory * > * trajectories = new std::vector < Trajectory* >;
	std::vector < Trajectory * > * new_trajectories;
	
	std::vector<double> * proj_latit_prev = new std::vector<double>;
	std::vector<double> * proj_longit_prev = new std::vector<double>;
	std::vector<double> * proj_latit_up;
	std::vector<double> * proj_longit_up;
	
	Trajectory* traj;
	double proj_latit;
	double proj_longit;
	double dist;

	trajectories->reserve(seg_probs.size());
	proj_latit_prev->reserve(seg_probs.size());
	proj_longit_prev->reserve(seg_probs.size());
	
	for(unsigned int c = 0; c < seg_probs.size(); c++)
	{
		net->closest_point_segment
			(
				seg_probs.at(c)->first,
				up->latit,
				up->longit,
				proj_latit,
				proj_longit
			);

		proj_latit_prev->push_back(proj_latit);
		proj_longit_prev->push_back(proj_longit);
		
		traj = new Trajectory;
		
		dist = std_distance
			(
				proj_latit, 
				net->seg_latit_begin(seg_probs.at(c)->first), 
				proj_longit,
				net->seg_longit_begin(seg_probs.at(c)->first)
			);

		traj->add_update(seg_probs.at(c)->first, up->time, dist, up);
		traj->prob = -1 * log(seg_probs.at(c)->second);
		trajectories->push_back(traj);

		delete seg_probs.at(c);
	}

	double max_prob;
	unsigned int max_traj;
	double prob;
	unsigned int u = 1;
	double trans_p;
	update* prev_up;
	unsigned int seg;
	unsigned int prev_seg;
	double gcd;
	
	++it;
	
	//Iterating over updates
	for(; it != updates.end(); ++it)
	{
		prev_up = up;
		up = *it;
		
		//Distance between updates
		gcd = std_distance(prev_up->proj_latit, up->proj_latit, 
			prev_up->proj_longit, up->proj_longit);
	
		//Identifying candidate road segments for update
		cand_seg_probs(seg_probs, up, net);
		new_trajectories = new std::vector < Trajectory * >;
		new_trajectories->reserve(seg_probs.size());
		
		proj_latit_up = new std::vector<double>;
		proj_longit_up = new std::vector<double>;
		
		proj_latit_up->reserve(seg_probs.size());
		proj_longit_up->reserve(seg_probs.size());

		for(unsigned int s = 0; s < seg_probs.size(); s++)
		{
			seg = seg_probs.at(s)->first;
			max_prob = std::numeric_limits<double>::max();
			max_traj = 0;

			net->closest_point_segment
				(
					seg,
					up->latit,
					up->longit,
					proj_latit,
					proj_longit
				);
			
			proj_latit_up->push_back(proj_latit);
			proj_longit_up->push_back(proj_longit);

			for(unsigned int t = 0; t < trajectories->size(); t++)
			{
				traj = trajectories->at(t);
				prev_seg = traj->back()->segment;
				
				//Computing transition probabilities
				
				prob = traj->prob - log(seg_probs.at(s)->second);
				
				if(prob - log(1) <= max_prob)
				{
					trans_p = transition_prob
						(
							prev_seg, 
							seg, 
							proj_latit_prev->at(t),
							proj_latit, 
							proj_longit_prev->at(t), 
							proj_longit, 
							prev_up->time, 
							up->time, 
							gcd, 
							net
						);

					prob -= log(trans_p);
				
					if(prob <= max_prob)
					{
						max_prob = prob;
						max_traj = t;
					}
				}
			}

			try
			{
				//Coping trajectory from last iteration
				traj = new Trajectory(*(trajectories->at(max_traj)));
			}
			catch(const std::out_of_range& oor)
			{
				//FIXME: This part is not very stable yet.
				std::cout << "error *" << std::endl;
				std::cout << trajectories->size() << std::endl;
				std::cout << "max_traj = " << max_traj << std::endl;
				exit(1);
			}
			
			dist = std_distance
				(
					proj_latit, 
					net->seg_latit_begin(seg_probs.at(s)->first), 
					proj_longit,
					net->seg_longit_begin(seg_probs.at(s)->first)
				);

			//Adding new update to trajectory 
			traj->add_update(seg, up->time, dist, up);
			traj->prob = max_prob;
			
			//Inserting new computed trajectory
			new_trajectories->push_back(traj);

			delete seg_probs.at(s);
		}
		
		//Deleting data from the previous iteration
		for(unsigned int t = 0; t < trajectories->size(); t++)
		{
			traj = trajectories->at(t);
			prev_seg = traj->back()->segment;
			delete traj;
		}
		
		delete trajectories;
		trajectories = new_trajectories;

		delete proj_latit_prev;
		delete proj_longit_prev;
		proj_latit_prev = proj_latit_up;
		proj_longit_prev = proj_longit_up;

		u++;
	}
	
	delete proj_latit_prev;
	delete proj_longit_prev;

	max_prob = std::numeric_limits<double>::max();
	Trajectory* most_likely_traj = NULL;
	
	//Identifying the most likely trajectory
	for(unsigned int t = 0; t < trajectories->size(); t++)
	{
		traj = trajectories->at(t);
		
		if(traj->prob <= 
			max_prob)
		{
			max_prob = traj->prob;
			most_likely_traj = traj;
		}
	}
	
	Trajectory* res;
	
	//In case the most likely trajectory is NULL, 
	//all trajectories have 0 probability and there
	//is something wrong.
	if(most_likely_traj != NULL)
	{
		res = new Trajectory(*most_likely_traj);
	}
	else
	{
		//TODO: Add exception
		res = NULL;
		std::cout << "Map matching error," << 
			" most likely trajectory has" <<
			" 0 probability" << std::endl;
	}
	
	for(unsigned int t = 0; t < trajectories->size(); t++)
	{
		traj = trajectories->at(t);
		delete traj;
	}

	delete trajectories;
	
	return res;
}

void Trajectory::extend_traj_shortest_paths(const RoadNet*net)
{
	std::list<unsigned int> sp;
	std::list< seg_time* >::iterator st_next;
	double spv;
	double speed;
	
	for(std::list< seg_time* >::iterator st = seg_time_lst.begin();
		st != seg_time_lst.end(); ++st)
	{
		st_next = st;
		st_next++;
		
		if(st_next != seg_time_lst.end())
		{
			//Computes shortest path between two segments
			//Some trajectories might be disconnected due to
			//GPS errors.
			spv = net->shortest_path(sp, (*st)->segment, (*st_next)->segment);
			spv += net->segment_length((*st)->segment) - (*st)->dist;
			spv += (*st_next)->dist;

			speed = (double) spv / ((*st_next)->time - (*st)->time);
			
			if(speed <= MAXSPEED)
			{
				for(std::list<unsigned int>::iterator s = sp.begin(); s != sp.end(); ++s)
				{
					seg_time_lst.insert
						(
							st_next, 
							new_seg_time(*s, 0, 0)
						);
					
					size_traj++;
					++st;
				}
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
				size_traj--;
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

const unsigned int Trajectory::read_trajectories
	(
		std::list< Trajectory * >& trajectories,
		const std::string input_file_name, 
		const RoadNet* net
	)
		throw (std::ios_base::failure)
{
	std::map<std::string, Trajectory*> traj_map;
	std::ifstream traj_file(input_file_name.c_str(), std::ios::in);
	std::string line_str;
	std::vector< std:: string > line_vec;

	if(! traj_file.is_open())
	{
		std::cerr << "Error: Could not open trajectory file "
			<< input_file_name << std::endl << std::endl;
		
		return 0;
	}
	
	std::string id;
	std::string seg_name;
	unsigned int seg;
	unsigned int time;
	double dist;
	std::getline(traj_file, line_str);
	Trajectory* traj;
	unsigned int num_traj = 0;
	unsigned int i;
	
	while(! traj_file.eof())
	{
		line_vec = split(line_str,',');

		if(line_vec.size() < 4)
		{
			 std::cerr << "Error: Invalid trajectory file format, check the README file: "
			 	<< input_file_name << std::endl << std::endl;
			 traj_file.close();

			 return 0;
		}

		id = line_vec[0];

		if(traj_map.find(id) == traj_map.end())
		{
			traj = new Trajectory(id);

			traj_map.insert(std::pair<std::string, Trajectory*>
				(id, traj));

			num_traj++;
		}
		else
		{
			traj = traj_map.at(id);
		}
		
		i = 1;
		for(; i < line_vec.size() - 3; i++)
		{
			seg_name = line_vec[i];
			seg = net->seg_ID(seg_name);
			traj->add_update(seg, 0, 0);
		}
		
		seg_name = line_vec[i];
		seg = net->seg_ID(seg_name);
		std::stringstream ss(line_vec[i+1]);
		ss >> time;
		std::stringstream se_dist(line_vec[i+2]);
		se_dist >> dist;
		traj->add_update(seg, time, dist);
		
		std::getline(traj_file, line_str);
	}

	traj_file.close();

	for(std::map<std::string,Trajectory*>::iterator it = traj_map.begin();
		it != traj_map.end(); ++it)
	{
		traj = it->second;
		trajectories.push_back(traj);
	}

	return num_traj;
}

void Trajectory::write(std::ofstream& output_file, const RoadNet* net) const
{
	seg_time* st;

	std::list < Trajectory* > decomp;
	decompose_online(decomp);
	Trajectory* traj;

	for(std::list < Trajectory* >::iterator itt = decomp.begin();
		itt != decomp.end(); itt++)
	{
		traj = *itt;
		output_file << obj << ",";
		
		for(std::list< seg_time* >::const_iterator its = traj->seg_time_lst.begin();
			its != traj->seg_time_lst.end(); ++its)
		{
			st = *its;
			output_file << net->seg_name(st->segment) << ",";
		}
		
		output_file << traj->back()->time << "," <<
			traj->back()->dist << "\n";
	}

	delete_trajectories(&decomp);

	output_file.flush();
}

const bool Trajectory::write_map_matched_trajectories
	(
		const std::string input_file_name, 
		const std::string output_file_name, 
		RoadNet* net,
		const double _sigma, 
		const double _radius, 
		const double _beta_const,
		const double _time_div
	)
{
	//Setting some variables used in the map-matching.
	sigma = _sigma;
	radius = _radius;
	beta_const = _beta_const;
	time_div = _time_div;

	std::vector<std::list< update* > * > updates;
	read_updates(updates, input_file_name, net);
	Trajectory* traj;
	std::ofstream output_file(output_file_name.c_str(), std::ios::out);

	if(! output_file.is_open())
	{
		std::cerr << "Error: Could not open trajectory file for writing: "
			<< output_file_name << std::endl << std::endl;
	     	
		return false;
	}
	
	//Precomputing shortest paths
	net->precompute_shortest_paths(MAXLENGTHSHORTESTPATH, num_threads);

	for(unsigned int u = 0; u < updates.size(); u++)
	{
		//Map-matching updates
		traj = map_matching(*(updates.at(u)), net);
		traj->write(output_file, net);
		delete traj;
		delete_list_updates(updates.at(u));
		delete updates.at(u);
	}

	output_file.close();
	net->clear_distances();

	return true;
}

void run_thread_map
	(
		std::vector<std::list< update* > * >* updates,
		unsigned int* pointer,
		pthread_mutex_t* mutex_pool,
		pthread_mutex_t* mutex_file,
		std::ofstream* output_file,
		RoadNet* net
	)
{
	std::list < update* >* object_updates;
	Trajectory* traj;

	while(true)
	{
		pthread_mutex_lock(mutex_pool);
		
		if(*pointer == updates->size())
		{
			pthread_mutex_unlock(mutex_pool);
			break;
		}
		else
		{
			object_updates = updates->at(*pointer);
			*pointer = *pointer + 1;
			pthread_mutex_unlock(mutex_pool);
		}
		
		traj = Trajectory::map_matching(*object_updates, net);
		traj->extend_traj_shortest_paths(net);
		traj->remove_repeated_segments();

		pthread_mutex_lock(mutex_file);
		traj->write(*output_file, net);
		pthread_mutex_unlock(mutex_file);
	}
}

void run_thread_exp
	(
		std::list< Trajectory* >*  trajectories,
		std::list< Trajectory* >::iterator* pointer,
		pthread_mutex_t* mutex_pool,
		RoadNet* net
	)
{
	Trajectory* traj;

	while(true)
	{
		pthread_mutex_lock(mutex_pool);
		
		if(*pointer == trajectories->end())
		{
			pthread_mutex_unlock(mutex_pool);
			break;
		}
		else
		{
			traj = *(*pointer);
			++(*pointer);
			pthread_mutex_unlock(mutex_pool);
		}
		
		traj->extend_traj_shortest_paths(net);
		traj->remove_repeated_segments();
	}
}

void* start_thread_map(void* v_param)
{
	p_thread_param_map* param = (p_thread_param_map*) v_param;
	run_thread_map(param->updates, param->pointer, param->mutex_pool,
		param->mutex_file, param->output_file, param->net);
	pthread_exit(NULL);
}

void* start_thread_exp(void* v_param)
{
	p_thread_param_exp* param = (p_thread_param_exp*) v_param;
	run_thread_exp(param->trajectories, param->pointer, param->mutex_pool,
		param->net);
	pthread_exit(NULL);
}

const bool Trajectory::write_map_matched_trajectories_multithreads
	(
		const std::string input_file_name, 
		const std::string output_file_name, 
		RoadNet* net,
		const double _sigma, 
		const double _radius, 
		const double _beta_const,
		const double _time_div
	)
{
	//Setting some variables used in the map-matching.
	sigma = _sigma;
	radius = _radius;
	beta_const = _beta_const;
	time_div = _time_div;

	std::vector<std::list< update* > * > updates;
	read_updates(updates, input_file_name, net);
	std::ofstream output_file(output_file_name.c_str(), std::ios::out);

	if(! output_file.is_open())
	{
		std::cerr << "Error: Could not open trajectory file for writing: "
			<< output_file_name << std::endl << std::endl;
	     	
		return false;
	}

	net->precompute_shortest_paths(MAXLENGTHSHORTESTPATH, num_threads);

	pthread_t* threads = (pthread_t*) malloc (num_threads * sizeof(pthread_t));
	p_thread_param_map* param;
	pthread_mutex_t* mutex_pool = new pthread_mutex_t;
	pthread_mutex_init(mutex_pool, NULL);
	pthread_mutex_t* mutex_file = new pthread_mutex_t;
	pthread_mutex_init(mutex_file, NULL);
	unsigned int pointer = 0;
	std::vector<p_thread_param_map*> params;

	for(unsigned int t = 0; t < num_threads; t++)
	{
		param = new_p_thread_param_map(&updates, &pointer, mutex_pool, mutex_file, &output_file, net);
		params.push_back(param);
		pthread_create(&threads[t], NULL, start_thread_map, param);
	}

	for(unsigned int t = 0; t < num_threads; t++)
	{
		pthread_join(threads[t], NULL);
	}
	 
	for(unsigned int t = 0; t < num_threads; t++)
	{
		delete params[t];
	}
	
	free(threads);
	delete mutex_pool;
	delete mutex_file;

	for(unsigned int u = 0; u < updates.size(); u++)
	{
		delete_list_updates(updates.at(u));
		delete updates.at(u);
	}

	output_file.close();
	net->clear_distances();

	return true;
}

void Trajectory::expand_trajectories
	(
		std::list< Trajectory * >& trajectories,
		RoadNet* net
	)
{
	pthread_t* threads = (pthread_t*) malloc (num_threads * sizeof(pthread_t));
	p_thread_param_exp* param;
	pthread_mutex_t* mutex_pool = new pthread_mutex_t;
	pthread_mutex_init(mutex_pool, NULL);
	std::list < Trajectory* >::iterator pointer = trajectories.begin();
	std::vector<p_thread_param_exp*> params;

	for(unsigned int t = 0; t < num_threads; t++)
	{
		param = new_p_thread_param_exp(&trajectories, &pointer, mutex_pool, net);
		params.push_back(param);
		pthread_create(&threads[t], NULL, start_thread_exp, param);
	}
	
	for(unsigned int t = 0; t < num_threads; t++)
	{
		pthread_join(threads[t], NULL);
	}
	 
	for(unsigned int t = 0; t < num_threads; t++)
	{
		delete params[t];
	}
	
	free(threads);
	delete mutex_pool;
}

const unsigned int Trajectory::read_updates
	(
		std::vector<std::list< update* > * >& updates,
		const std::string input_file_name,
		const RoadNet* net
	) 
		throw (std::ios_base::failure)
{
	std::ifstream input_file(input_file_name.c_str(), std::ios::in);
	std::string line_str;
	std::vector< std:: string > line_vec;
	 
	if(! input_file.is_open())
	{
		std::cerr << "Error: Could not open gps updates file "
			<< input_file_name << std::endl << std::endl;
		
		return 0;
	}

	std::map<std::string, unsigned int> object_ids;
	std::string id;
	std::getline(input_file, line_str);
	unsigned int num_id = 0;
	
	while(! input_file.eof())
	{
		line_vec = split(line_str,',');

		if(line_vec.size() < 5)
		{
			 std::cerr << "Error: Invalid trajectory file format, check the README file"
			 	<< input_file_name << std::endl << std::endl;
			 input_file.close();

			 return 0;
		}

		id = line_vec[0];
		
		if(object_ids.find(id) == object_ids.end())
		{
			object_ids[id] = num_id;
			num_id++;
		}

		std::getline(input_file, line_str);
	}

	input_file.clear();
	input_file.seekg(0);
		
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

	std::getline(input_file, line_str);

	while(! input_file.eof())
	{
		line_vec = split(line_str,',');

		if(line_vec.size() < 5)
		{
			 std::cerr << "Error: Invalid trajectory file format, check the README file"
			 	<< input_file_name << std::endl << std::endl;
			 input_file.close();

			 return 0;
		}

		id = line_vec[0];
		latit = atof(line_vec[1].c_str());
		longit = atof(line_vec[2].c_str());
		std::stringstream ss(line_vec[4]);
		ss >> timestamp;

		num_id = object_ids[id];
		
		net->project(latit, longit, proj_latit, proj_longit);
		updates.at(num_id)->push_back(new_update(id, latit, 
			longit, proj_latit,
			proj_longit, timestamp));

		std::getline(input_file, line_str);
	}

	input_file.close();

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
		if((*it)->up == NULL)
		{
			std::cout << "(" << (*it)->segment << "," << (*it)->time << "," << (*it)->dist << ") ";
		}
		else
		{
			std::cout << "(" << (*it)->up->object << "," << (*it)->segment << "," << (*it)->time << "," << (*it)->dist << ") ";
		}
	}

	std::cout << std::endl;
}

void Trajectory::decompose_online(std::list<Trajectory*>& decomp) const
{
	seg_time* st;
	decomp.push_back(new Trajectory);

	for(std::list< seg_time* >::const_iterator it = seg_time_lst.begin();
		it != seg_time_lst.end(); ++it)
	{
		st = *it;
		decomp.back()->add_update(st->segment, st->time, st->dist);
		
		if(st->time != 0 || st->dist != 0)	//update, i.e. not shortest path completion
		{
			decomp.push_back(new Trajectory);
		}
	}

	decomp.pop_back();
}

void Trajectory::decompose_delay
	(
		std::list<Trajectory*>& decomp, 
		const unsigned int delay,
		RoadNet* net
	) const
{
	seg_time* st;
	dist_time* dt;
	decomp.push_back(new Trajectory);
	std::list < dist_time* > dist_times;
	get_dist_times_uniform(dist_times, net);

	std::list< seg_time* >::const_iterator itraj = seg_time_lst.begin();
	std::list< dist_time* >::const_iterator idt = dist_times.begin();
	unsigned int start_time = dist_times.front()->time;

	while(itraj != seg_time_lst.end())
	{
		st = *itraj;
		dt = *idt;

		if(dt->time - start_time > delay)
		{
			start_time = dt->time;
			decomp.push_back(new Trajectory);
		}	
		
		decomp.back()->add_update(st->segment, st->time, st->dist);
		
		++itraj;
		++idt;
	}
}

void Trajectory::append(Trajectory* traj)
{
	seg_time* st;

	for(std::list< seg_time* >::const_iterator it = seg_time_lst.begin();
		it != seg_time_lst.end(); ++it)
	{
		st = *it;
		add_update(st->segment, st->time, st->dist);
	}	
}

void delete_list_updates(std::list<update*>* updates)
{
	for(std::list<update*>::iterator it = updates->begin(); 
		it != updates->end(); ++it)
	{
		delete *it;
	}
}

const bool TrajDB::insert(const std::string& obj, Trajectory& traj)
{
	for(Trajectory::iterator it = traj.begin();
		it != traj.end(); ++it)
	{
		insert(obj, *(*it));
	}

	return true;
}

const bool TrajDB::insert(const std::string& input_file_name)
{
	std::list<Trajectory*> trajectories;
	std::string obj;
	Trajectory* traj;

	if(Trajectory::read_trajectories(trajectories, input_file_name, net))
	{
		for(std::list<Trajectory*>::iterator it = trajectories.begin();
			it != trajectories.end(); ++it)
		{
			traj = *it;
			insert(traj->object(), *traj);
		}

		Trajectory::delete_trajectories(&trajectories);
		
		return true;
	}
	else
	{
		return false;
	}
}

const bool TrajDB::insert
	(
		const std::string& obj,
		const seg_time& st
	)
{
	n_updates++;
	return db->insert(obj, st);
}

const bool TrajDB::center_radius_query
	(
		const double latit,
		const double longit,
		const double dist,
		std::list<std::string>& res,
		const unsigned int time_begin,
		const unsigned int time_end
	)
	const
{
	std::list<unsigned int> segs;
	net->segments_within_distance(segs, latit, longit, dist);
	res.clear();

	std::list<std::string> objs;

	for(std::list<unsigned int>::iterator it = segs.begin();
		it != segs.end(); ++it)
	{
		db->query_segment_time(*it, objs, time_begin, time_end);
		res.splice(res.end(), objs);
	}

	res.sort();
	res.unique();

	return true;
}

const bool TrajDB::nearest_neighbor_query
	(
		const double lat,
		const double longit,
		const std::string& res,
		const unsigned int time_begin,
		const unsigned int time_end
	)
	const
{
	return false;
}

const bool TrajDB::when_at
	(
		const std::string& obj,
		const double lat,
		const double longit,
		const unsigned int& timestamp
	)
		const
{
	return false;
}

const bool TrajDB::where_at
	(
		const std::string& obj,
		const unsigned int timestamp,
		double& lat,
		double& longit
	)
	const
{
	return false;
}

//const bool TrajDBPostGis::connect()
//{
//	std::string conn_str = "dbname=" + database_name
//		+ " user=" + user
//		+ " password=" + password
//		+ " hostaddr=" + host
//		+ " port=" + port;
	       
//	try
//	{
//		conn = new pqxx::connection(conn_str.c_str());
	   
//		if(conn->is_open())
//		{
//			return true;
//		}
//		else
//	        {
//			return false;
//		}
//	}
//	catch(const pqxx::broken_connection &e)
//	{
//		std::cerr << "Error: Cannot connect to the database "
//			<< database_name << " with user "
//			<< user << " host " << host << " port "
//			<< port << std::endl << std::endl;
//		std::cerr << conn_str << std::endl;
//		std::cerr << e.what() << std::endl;
			
//		return false;
//	}
				        
//	return true;
//}

//const bool TrajDBPostGis::create()
//{
//	std::string sql;
	  
//	try
//	{
		//CREATE TABLE table_name (modid varchar(60) PRIMARY KEY, 
		//	startime timestamp(5), endtime timestamp(5), 
		//	flag smallint, linkid integer
		//	speed real, anchorpoint geography(POINT,4326));
		//CREATE INDEX traj_modid_idx_table_name ON table_name USING HASH(modid);
		//CREATE INDEX traj_linkid_idx_table_name ON table_name USING HASH(linkid);
		//CREATE INDEX traj_startendtime_idx_table_name ON table_name(starttime,endtime);
//		sql = "CREATE TABLE " + table_name +
/*			"(modid varchar(60), starttime timestamp(5),\
			endtime timestamp(5), flag smallint, linkid integer, speed real,\
			anchorpoint geography(POINT," + spatial_ref + "));\
			CREATE INDEX traj_modid_idx_" + table_name + " ON " + table_name + " USING HASH(modid);\
			CREATE INDEX traj_linkid_idx_" + table_name + " ON " + table_name + " USING HASH(linkid);\
			CREATE INDEX traj_startendtime_idx_" + table_name +" ON " + table_name + "(starttime,endtime);";
*/		
//		pqxx::work work (*conn);
//		work.exec(sql.c_str());
//		work.commit();
//	}
//	catch(const pqxx::sql_error& e)
//	{
//		std::cerr << "Error: Failed query:" << std::endl;
//		std::cerr << sql << std::endl;
//		std::cerr << e.what() << std::endl;

//		return false;
//	}

//	return true;
//}

//const bool TrajDBPostGis::drop()
//{
//	std::string sql = "DROP TABLE " + table_name + ";";

//	try
//	{
//		sql = "DROP TABLE " + table_name + ";";
//		pqxx::work work (*conn);
//		work.exec(sql.c_str());
//		work.commit();
//	}
//	catch(const pqxx::sql_error& e)
//	{
//		std::cerr << "Error: Failed query: " << std::endl;
//		std::cerr << sql << std::endl;
//		std::cerr << e.what() << std::endl;
		
//		return false;
//	}

//	return true;
//}

//const bool TrajDBPostGis::insert
//	(
//		const std::string& obj,
//		const seg_time& st
//	)
//{
//	std::string sql;
//	n_updates++;
	//FIXME: Some variables need to be set properly
//	unsigned int flag = 0;
//	double speed = 0;

//	try
//	{
//		sql = "INSERT INTO " + table_name +
/*			"(modid, starttime, endtime, flag, linkid, speed, anchorpoint)\
			VALUES ('" + obj + "'," +
			"TO_TIMESTAMP(" + to_string(st.start_time) + ")," +
			"TO_TIMESTAMP(" + to_string(st.end_time) + ")," +
			to_string(flag) + "," +
			to_string(st.segment) + "," +
			to_string(speed) + "," +
			"NULL);";
*/		
//		pqxx::work work (*conn);
//		work.exec(sql.c_str());
//		work.commit();
//	}
//	catch(const pqxx::sql_error& e)
//	{
//		std::cerr << "Error: Failed query:" << std::endl;
//		std::cerr << sql << std::endl;
//		std::cerr << e.what() << std::endl;
 
//		return false;
//	}
	
//	return true;
//}

//const bool TrajDBPostGis::query_segment_time
//	(
//		const unsigned int segment,
//		std::list<std::string>& objs,
//		const unsigned int time_begin,
//		const unsigned int time_end
//	)
//	const
//{
//	std::string sql;

//	try
//	{
//		if(time_begin != 0 && time_end != 0)
//		{
//			sql = "SELECT modid FROM " + table_name + 
//				" WHERE linkid=" + to_string(segment) +
//				" AND starttime < TO_TIMESTAMP(" + to_string(time_end) + ")" +
//				 " AND endtime > TO_TIMESTAMP(" + to_string(time_begin) + ")';";
//		} 
//		else
//		{
//			sql = "SELECT modid FROM " + table_name + 
//				" WHERE linkid=" + to_string(segment) + ";";
//		}

//		pqxx::nontransaction work(*conn);
//		pqxx::result res(work.exec(sql.c_str()));
//		objs.clear();
		       
//		for (pqxx::result::const_iterator r = res.begin(); r != res.end(); ++r)
//		{
//			objs.push_back(r[0].as<std::string>());
//		}
//	}
//	catch(const pqxx::sql_error& e)
//	{
//		std::cerr << "Error: Failed query: " << std::endl;
//		std::cerr << sql << std::endl;
//		std::cerr << e.what() << std::endl;
//
//		return false;
//	}

//	return true;
//}

