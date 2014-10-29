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
 *	FILE moving_obj.h: Moving object processing
**/

#ifndef MOVINGOBJ_H
#define MOVINGOBJ_H

/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <iostream>

#include "road_net.h"

typedef struct t_update
{
	double latit;
	double longit;
	double proj_latit;
	double proj_longit;
	std::string object;
	unsigned int time;
}update;

update* new_update(const std::string obj, const double latit, const double longit, unsigned int time);
update* new_update(const std::string obj, const double latit, const double longit, const double proj_latit, 
	const double proj_longit, unsigned int time);

typedef struct t_seg_time
{
	unsigned int segment;
	unsigned int time;
}seg_time;

seg_time* new_seg_time(const unsigned int segment, const unsigned int time);

/**
 * Trajectory
**/
class Trajectory
{
	public:
		Trajectory();

		Trajectory(const Trajectory& traj);
		
		virtual ~Trajectory();

		inline const unsigned int size()
		{
			return size_traj;
		}

		void add_update(const unsigned int segment, const unsigned int timestamp);
		
		static const unsigned int build_trajectories(std::list<Trajectory*>& trajectories, 
			const std::list<update*>& updates, const RoadNet* net);
		
		static std::list<Trajectory*>* read_trajectories(const std::string input_file_name, 
			const RoadNet* net) throw (std::ios_base::failure);
		static void delete_trajectories(std::list<Trajectory*>* trajectories);

		void print()  const;
		
		typedef std::list< seg_time* >::iterator iterator;
		
		inline iterator begin()
		{
			return seg_time_lst.begin();
		}

		inline iterator end() 
		{
			return seg_time_lst.end();
		}

		inline seg_time* back()
		{
			return seg_time_lst.back();
		}
	private:
		static const double max_speed;
		static const double min_speed;
		std::list< seg_time* > seg_time_lst;
		unsigned int size_traj;
		double prob;

		static const double time_div;
		static const double beta_const;
		static const double sigma;

		void regression_timestamps(const RoadNet* net);
		static const double transition_prob(const unsigned int seg_from, 
			const unsigned int seg_to,
			const double latit_from, const double latitt_to,
			const double longit_from, const double longit_to,
			const unsigned int time_from, const unsigned int time_to,
			const RoadNet* net);
		static std::vector < std::pair < unsigned int, double > * >
			cand_seg_probs(const update* up, const RoadNet* net);
};

#endif
