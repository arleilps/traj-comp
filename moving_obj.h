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

/**
 * GPS update
**/
typedef struct t_update
{
	double latit;
	double longit;
	double proj_latit;
	double proj_longit;
	std::string object;
	unsigned int time;
} update;

/**
 * Creates new GPS update.
**/
update* new_update
	(
		const std::string obj, 
		const double latit, 
		const double longit, 
		unsigned int time
	);

/**
 * Creates new GPS update.
**/
update* new_update
	(
		const std::string obj, 
		const double latit, 
		const double longit, 
		const double proj_latit, 
		const double proj_longit, 
		unsigned int time
	);

/**
 * Deletes updates in a list.
**/
void delete_list_updates(std::list<update*>* updates);

/**
 * Map-matched update (segment,time)
**/
typedef struct t_seg_time
{
	unsigned int segment;
	unsigned int time;
	const update* up;
} seg_time;

/**
 * Creates new map-matched trajectory update.
**/
seg_time* new_seg_time
	(
		const unsigned int segment, 
		const unsigned int time, 
		const update* up=NULL
	);

/**
 * Implements functionalities for trajectory manipulation (e.g. creation, map-matching etc.)
**/
class Trajectory
{
	public:
		/*CONSTRUCTORS*/

		/**
		 * Creates an empty trajectory
		**/
		Trajectory();

		/**
		 * Copy constructor
		**/
		Trajectory(const Trajectory& traj);
		
		/**
		 * Destructor
		**/
		virtual ~Trajectory();
		
		 /*STATIC METHODS*/
		 
		 /**
		  * Performs map-matching using Hidden markov model and viterbi algorithm.
		  * Based on paper: An HMM-based Map Matching Method with Cumulative 
		  * Proximity-Weight Formulation
		  * @param updates gps updates (lat-long) to be matched
		  * @param net road network structure
		  * @return map-matched trajectory
		 **/
		static Trajectory* map_matching
			(
				const std::list<update*>& updates, 
				RoadNet* net
			);
		
		/**
		 * Reads gps updates from an input file. See README for formatting.
		 * @param updates updates list to be filled
		 * @param input_file_name input file
		 * @param net road network structure
		 * @return number of different objects for which the updates were read
		 * @throws ios_base::failure if has problems opening or reading file
		**/
		static const unsigned int read_updates
			(
				std::vector<std::list< update* > * >& updates, 
				const std::string input_file_name, 
				const RoadNet* net
			) 
				throw (std::ios_base::failure);
		/**
		 * Deletes trajectories from a list.
		 * @param trajectories list of trajectories to be deleted
		**/
		static void delete_trajectories
			(
				std::list<Trajectory*>* trajectories
			);
		
		/**
		 * Reads map-matched trajectories from an input file.
		 * We assume that this program generated the trajectories
		 * so the input-output formats should be compatible.
		 * @param trajectories trajectories list to be filled
		 * @param input_file_name input file
		 * @param net road network structure
		**/
		static const unsigned int read_trajectories
			(
				std::list< Trajectory * >& trajectories, 
				const std::string input_file_name, 
				const RoadNet* net
			) 
				throw (std::ios_base::failure);
		/**
		 * Map-matches trajectories from an input file and writes
		 * them into an output file.
		 * @param input_file_name input file with gps updates
		 * @param output_file_name output file 
		 * @param net road network structure
		 * @param _sigma standard deviation of the gps data
		 * @param _radius radius*sigma distance to be checked for matching segments
		 * @param _beta_const constant beta for matching (see paper)
		 * @param _time_div another constant for matching (see paper)
		 * @return true in case of success, false otherwise
		**/
		static const bool write_map_matched_trajectories
			(
				const std::string input_file_name,
				const std::string output_file_name,
				RoadNet* net,
				const double _sigma=SIGMA,
				const double _radius=RADIUS,
				const double _beta_const=BETACONST,
				const double _time_div=TIMEDIV
			);

		 /*OBJECT METHODS*/
		
		/**
		 * Adds a map-matched update to the trajectory
		 * @param segment 
		 * @param timestamp
		 * @param update update that was matched
		**/
		void add_update
			(
				const unsigned int segment, 
				const unsigned int timestamp, 
				const update* up=NULL
			);
		
		/**
		 * Writes a trajectory into a file.
		 * Format <object, segment, timestamp>, one per line.
		 * @param output_file output file
		 * @param net road network structure
		**/
		void write
			(
				std::ofstream& output_file, 
				const RoadNet* net
			) 
				const;

		/**
		 * Prints a trajectory on the screen (for testing).
		**/
		void print()  const;
		
		/**
		 * Extends a trajectory assuming objects move in 
		 * shortest paths.
		 * @param net road network structure
		**/
		void extend_traj_shortest_paths(const RoadNet* net);

		/**
		 * Removes repeated segments from a trajectory.
		**/
		void remove_repeated_segments();

		/*PUBLIC VARIABLES*/
		
		//iterator for trajectories
		typedef std::list< seg_time* >::iterator iterator;
		
		/*INLINES*/

		/**
		 * Iterator to the beginning of the trajectory
		**/
		inline iterator begin()
		{
			return seg_time_lst.begin();
		}

		/**
		 * Iterator to the end of the trajectory
		**/
		inline iterator end() 
		{
			return seg_time_lst.end();
		}
		
		/**
		 * Iterator to the last segment of the trajectory
		**/
		inline seg_time* back()
		{
			return seg_time_lst.back();
		}
		
		/**
		 * Gives the size of a trajectory
		**/
		inline const unsigned int size()
		{
			return size_traj;
		}
	private:
		/*OBJECT VARIABLES*/

		std::list< seg_time* > seg_time_lst;
		unsigned int size_traj;
		double prob;
		
		/*STATIC VARIABLES*/

		static double time_div;
		static double beta_const;
		static double sigma;
		static double radius;
		
		/*CONSTANTS*/

		static const double TIMEDIV;
		static const double BETACONST;
		static const double SIGMA;
		static const double RADIUS;
		
		/*OBJECT METHODS*/

		static const double transition_prob
			(
				const unsigned int seg_from, 
				const unsigned int seg_to,
				const double latit_from, 
				const double latitt_to,
				const double longit_from, 
				const double longit_to,
				const unsigned int time_from, 
				const unsigned int time_to,
				const double gcd,
				const RoadNet* net
			);
		
		static void cand_seg_probs
			(
				std::vector < std::pair < unsigned int, double > * >& seg_probs, 
				const update* up, const RoadNet* net
			);
};

#endif
