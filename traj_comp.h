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
 *	FILE traj_comp.h: Trajectory compression algorithms
**/

#ifndef TRAJCOMP_H
#define TRAJCOMP_H

/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <iostream>

/*my includes*/
#include "road_net.h"
#include "moving_obj.h"

/*Node in the frequent subtrajectory tree*/
typedef struct t_node
{
	unsigned int id;
	unsigned int seg;
	unsigned int freq;
	unsigned int depth;
	std::map<unsigned int, t_node*>* children;
	t_node* suffix;
}Node;

/**
 * Implements functionalities for a compressed trajectory manipulation
 * so far, nothing beyond standard trajectories (see moving_obj.h)
**/
class CompTrajectory: public Trajectory
{
	public:
		CompTrajectory():Trajectory(){};
		CompTrajectory(const CompTrajectory& traj):Trajectory(traj){};
};

/**
 * Generic trajectory compression algorithm
**/
class TrajCompAlgo
{
	public:
		/*Constructor*/
		TrajCompAlgo(RoadNet* _net);
		
		/*Destructor*/
		virtual ~TrajCompAlgo();
		
		/** 
		 * Trains the compression algorithm using map-matched trajectories in an 
		 * input file.
		 * @param training_traj_file_name training file name
		 * @return size of the model
		**/
		virtual const unsigned int train(const std::string training_traj_file_name)
		{
			return 0;
		}
		
		/** 
		 * Tests the compression algorithm using map-matched trajectories in an 
		 * input file.
		 * @param test_traj_file_name test file name
		 * @return total number of updates made
		**/
		virtual const unsigned int test(const std::string test_traj_file_name)
		{
			return 0;
		}
		
		/**
		 * Inserts a single update to the trajectory compression model.
		 * Useful for real-time compression.
		 * @param obj object id
		 * @param seg segment id
		 * @param timestamp time
		**/
		virtual void update(const unsigned int& obj, const unsigned int& seg, 
			const unsigned int timestamp){};
		
		/**
		 * Compresses an input trajectory.
		 * @param traj trajectory
		 * @return compressed trajectory
		**/
		virtual CompTrajectory* compress(Trajectory* traj) const
		{
			return NULL;
		}
	protected:
		RoadNet* net;
};

/**
 * Implements trajectory compression algorithm using frequent subtrajectories.
 * Works in batch-mode.
**/
class FreqSubt: public TrajCompAlgo
{
	public:
		/**
		 * Constructor.
		 * @param _min_sup minimum support for frequent subtrajectories
		 * @param _max_length maximum length for frequent subtrajectories
		 * @param net road network
		**/
		FreqSubt
			(
				const double _min_sup, 
				const unsigned int _max_length, 
				RoadNet* net
			)
			:TrajCompAlgo(net)
		{
			min_sup = _min_sup;
			max_length = _max_length;
			tree = new_node();
			Node* node;
			size_tree = 0;
			seg_to_freq_subt_index.reserve(net->size());

			for(unsigned int s = 0; s < net->size(); s++)
			{
				seg_to_freq_subt_index.push_back(new std::list<unsigned int>);
				size_tree++;
				node = new_node();
				node->seg = s;
				node->freq = 0;
				node->depth = 1;
				tree->children->insert(std::pair<unsigned int, Node*>(s, node));
			}
		}
		
		/*Destructor*/
		virtual ~FreqSubt()
		{
			delete_tree(tree);
		}

		const unsigned int train(const std::string training_traj_file_name);

		const unsigned int test(const std::string test_traj_file_name);

		void add_trajectory(Trajectory* traj);
		
		/**
		 * Gets list of frequent subtrajectories.
		 * @param fsts list to be updated with frequent subtrajectories
		**/
		void freq_sub_traj
			(
				std::list<std::pair<unsigned int, Trajectory * > * >& fsts
			);
		
		/**
		 * Prints frequent subtrajectory tree.
		**/
		void print();
		
		CompTrajectory* compress(Trajectory* traj) const;

		/**
		 * Updates list with all frequent subtrajectories that contain a given segment.
		 * @param seg segment
		 * @param freq_subt_ids list to be updated with frequent subtrajectory ids
		**/
		void get_freq_subt_ids
			(
				const unsigned int seg, 
				std::list<unsigned int>& freq_subt_ids
			)
			const;
	private:
		/* OBJECT VARIABLES */
		double min_sup;
		unsigned int max_length;
		Node* tree;
		unsigned int size_tree;
		unsigned int id;
		std::vector< std::list<unsigned int>* > seg_to_freq_subt_index;
		
		/* OBJECT METHODS */
		void print_tree(Node* node);
		void print_tree(Node* node, const std::string str);
		void delete_tree(Node* node);
		
		void freq_sub_traj
			(
				std::list<std::pair<unsigned int, Trajectory * > * >& fsts,
				Node* node, 
				Trajectory* traj=NULL
			);

		void prune_unfrequent_subtraj();
		void prune_tree(Node* root);
		Node* new_node();
		void set_seg_index();
		void set_seg_index(Node* root);
		
		void add_trajectory
			(
				Trajectory::iterator it, 
				Trajectory* traj, 
				Node* tree,
				const unsigned int depth,
				std::list<Node*>* suffix_pointers,
				std::list<Node*>* new_suffix_pointers
			);
};

/**
 * Implements database functionalies for trajectories using frequent subtrajectory compression.
**/
class FreqSubtCompTrajDB:public TrajDB
{
	public:
		/**
		 * Constructor.
		 * @param train_file training file
		 * @param min_sup minimum support for frequent subtrajectories
		 * @param max_length maximum length for frequent subtrajectories
		 * @param net road network
		**/
		FreqSubtCompTrajDB
			(
				const std::string& train_file,
				const double min_sup,
				const double max_length,
				RoadNet* net
			):TrajDB(net)
		{
			alg = new FreqSubt(min_sup, max_length, net);
			alg->train(train_file);
		}

		/*Destructor*/
		virtual ~FreqSubtCompTrajDB()
		{
			delete alg;
		}
		
		/**
		 * Gets list of frequent subtrajectories.
		 * @param fsts list to be updated with frequent subtrajectories
		**/
		inline void freq_sub_traj
			(
				std::list<std::pair<unsigned int, Trajectory * > * >& fsts
			)
		{
			alg->freq_sub_traj(fsts);
		}
		
		/*Methods from TrajDB (see moving_obj.h)*/

		const bool insert(const std::string& input_file_name);

		const bool insert(const std::string& obj, Trajectory& traj);
		
		const bool insert
			(
				const std::string& obj,
				const seg_time& st
			);

		const bool center_radius_query
			(
				const double lat,
				const double longit,
				const double dist,
				std::list<std::string>& res,
				const unsigned int time_begin=0,
				const unsigned int time_end=0
			)
				const;
	private:
		FreqSubt* alg;
};
#endif

