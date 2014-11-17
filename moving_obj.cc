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

//Default values for constants used in map-matching
const double Trajectory::TIMEDIV = 10;
const double Trajectory::SIGMA = 4.07;
const double Trajectory::BETACONST = 1;
const double Trajectory::RADIUS = 10;


double Trajectory::time_div = TIMEDIV;
double Trajectory::sigma = SIGMA;
double Trajectory::beta_const = BETACONST;
double Trajectory::radius = RADIUS;


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
		const update* up
	)
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

void Trajectory::add_update
	(
		const unsigned int segment, 
		const unsigned int timestamp, 
		const update* up
	)
{
	seg_time* st = new_seg_time(segment, timestamp, up);
	seg_time_lst.push_back(st);
	size_traj++;
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
}

/*
void Trajectory::pairwise_shortest_paths
	(
		std::list<unsigned int>& from, 
		std::list<unsigned int>& to,
		std::map<std::pair<unsigned int, unsigned int>, double>& distances,
		const RoadNet* net
	)
{
	for(std::list<unsigned int>::iterator it_from = from.begin();
		it_from != from.end(); ++it_from)
	{
		net->shortest_path(*it_from, to, latit_from, latit_to, longit_from, longit_to);
	}
}
*/

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
	double sd = net->shortest_path(seg_from, seg_to, 
		latit_from, latit_to, longit_from, longit_to);

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
/*
Trajectory* Trajectory::map_matching
	(
		const std::list<update*>& updates, 
		const RoadNet* net
	)
{
	std::list<update*>::const_iterator it = updates.begin();
	update* up = *it;
	std::vector < std::pair < unsigned int, double > * > seg_probs;
	
	//Identifying candidate road segments for update
	cand_seg_probs(seg_probs, up, net);
	
	std::vector< std::map < unsigned int, double > * > matrix_prob;
	std::vector< std::map < unsigned int, Trajectory* > * > matrix_traj;
	matrix_prob.reserve(updates.size());
	matrix_traj.reserve(updates.size());
	
	//Creating trajectory and probability matrix for viterbi algorithm
	for(unsigned int u = 0; u < updates.size(); u++)
	{
		matrix_prob.push_back(new std::map<unsigned int, double >);
		matrix_traj.push_back(new std::map<unsigned int, Trajectory* >);
	}

	Trajectory* traj;
	
	//Initializing matrices
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
	
	//Iterating over updates
	for(; it != updates.end(); ++it)
	{
		std::cout << "u = " << u << std::endl;
		prev_up = up;
		up = *it;
		
		//Identifying candidate road segments for update
		cand_seg_probs(seg_probs, up, net);
		
		for(unsigned int s = 0; s < seg_probs.size(); s++)
		{
			seg = seg_probs.at(s)->first;
			max_prob = std::numeric_limits<double>::max();
			max_seg = 0;

			for(it_traj = matrix_traj.at(u-1)->begin(); 
				it_traj != matrix_traj.at(u-1)->end(); ++it_traj)
			{
				traj = it_traj->second;
				prev_seg = it_traj->first;
				
				//Computing transition probabilities
				
				trans_p = transition_prob
					(
						prev_seg, 
						seg, 
						prev_up->latit, 
						up->latit, 
						prev_up->longit, 
						up->longit, 
						prev_up->time, 
						up->time, net
					);
			
				try
				{
					//Probabilities computed using logarithms
					//otherwise, they would approach to 0 for long
					//sequences
					prob = matrix_prob[u-1]->at(prev_seg) 
						- log(trans_p)  - log(seg_probs.at(s)->second);
				}
				catch(const std::out_of_range& oor)
				{
					//FIXME: This part is not very stable yet.
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
				//Coping trajectory from last iteration
				traj = new Trajectory(*(matrix_traj[u-1]->at(max_seg)));
			}
			catch(const std::out_of_range& oor)
			{
				//FIXME: This part is not very stable yet.
				std::cout << "error 2" << std::endl;
				std::cout << matrix_traj[u-1]->size() << std::endl;
				std::cout << "max_seg = " << max_seg << std::endl;
				std::cout << "m = " << (matrix_traj[u-1]->begin())->first << std::endl;
				exit(1);
			}

			//Adding new update to trajectory 
			traj->add_update(seg, up->time, up);
			traj->prob = max_prob;
			
			//Inserting new computed trajectory into the matrix
			matrix_traj[u]->insert(std::pair<unsigned int, Trajectory*>
				(seg, traj));

			delete seg_probs.at(s);
		}
		
		//Deleting data from the previous iteration
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
	
	//Identifying the most likely trajecotyr
	for(it_traj = matrix_traj[u-1]->begin(); 
		it_traj != matrix_traj[u-1]->end(); ++it_traj)
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
	
	for(it_traj = matrix_traj[u-1]->begin(); it_traj != matrix_traj[u-1]->end(); ++it_traj)
	{
		traj = it_traj->second;
		delete traj;
	}

	delete matrix_traj[u-1];
	delete matrix_prob[u-1];
	
	return res;
}
*/
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

	trajectories->reserve(seg_probs.size());
	proj_latit_prev->reserve(seg_probs.size());
	proj_longit_prev->reserve(seg_probs.size());
	
	for(unsigned int c = 0; c < seg_probs.size(); c++)
	{
		traj = new Trajectory;
		traj->add_update(seg_probs.at(c)->first, up->time, up);
		traj->prob = -1 * log(seg_probs.at(c)->second);
		trajectories->push_back(traj);

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
		std::cout << "u = " << u << std::endl;
		prev_up = up;
		up = *it;
		
		//Distance between updates
		//gcd = net->distance_points(prev_up->latit, 
		//	up->latit, prev_up->longit, up->longit);
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

			//Adding new update to trajectory 
			traj->add_update(seg, up->time, up);
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
			net->clear_saved_shortest_paths(prev_seg);
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
			net->shortest_path(sp, (*st)->segment, (*st_next)->segment, 
				(*st)->up->latit, (*st_next)->up->latit, 
				(*st)->up->longit, (*st_next)->up->longit);
			
			for(std::list<unsigned int>::iterator s = sp.begin(); s != sp.end(); ++s)
			{
				seg_time_lst.insert(st_next, new_seg_time(*s, (*st)->time, (*st)->up));
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
	unsigned int timestamp;
	std::getline(traj_file, line_str);
	Trajectory* traj;
	
	//First line contains number of trajectories
	unsigned int num_traj = atoi(line_str.c_str());
	
	while(! traj_file.eof())
	{
		line_vec = split(line_str,',');

		if(line_vec.size() < 3)
		{
			 std::cerr << "Error: Invalid trajectory file format, check the README file"
			 	<< input_file_name << std::endl << std::endl;
			 traj_file.close();

			 return 0;
		}

		id = line_vec[0];
		seg_name = line_vec[1];
		seg = net->seg_ID(seg_name);
		std::stringstream ss(line_vec[2]);
		ss >> timestamp;

		if(traj_map.find(id) == traj_map.end())
		{
			traj = new Trajectory();
			traj->add_update(seg, timestamp);
			traj_map.insert(std::pair<std::string, Trajectory*>
				(id, traj));
		}
		else
		{
			traj = traj_map.at(id);
			traj->add_update(seg, timestamp);
		}
		
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
	//TODO: Print more information here
	seg_time* st;
	for(std::list< seg_time* >::const_iterator it = seg_time_lst.begin();
		it != seg_time_lst.end(); ++it)
	{
		st = *it;
		output_file << st->up->object << "," << 
			net->seg_name(st->segment) << "," <<
			st->time << "\n";
	}
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

	output_file << updates.size() << "\n";

	for(unsigned int u = 0; u < updates.size(); u++)
	{
		//Map-matching updates
		traj = map_matching(*(updates.at(u)), net);
		traj->extend_traj_shortest_paths(net);
		traj->remove_repeated_segments();
		traj->write(output_file, net);
		delete traj;
		delete_list_updates(updates.at(u));
	}

	output_file.close();

	return true;
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
