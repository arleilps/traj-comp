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

/*Not in the frequent subtrajectory tree*/
typedef struct t_node
{
	unsigned int id;
	unsigned int seg;
	unsigned int freq;
	unsigned int depth;
	std::map<unsigned int, t_node*>* children;
	t_node* suffix;
}Node;

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
		
		virtual const unsigned int train(const std::string training_traj_file_name)
		{
			return 0;
		}

		virtual const unsigned int test(const std::string test_traj_file_name)
		{
			return 0;
		}

		virtual ~TrajCompAlgo();
		virtual void update(const unsigned int& obj, const unsigned int& seg, 
			const unsigned int timestamp){};
		
		virtual CompTrajectory* compress(Trajectory* traj) const
		{
			return NULL;
		}
	protected:
		RoadNet* net;
};

class FreqSubt: public TrajCompAlgo
{
	public:
		/*Constructor*/
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

			for(unsigned int s = 0; s < net->size(); s++)
			{
				size_tree++;
				node = new_node();
				node->seg = s;
				node->freq = 0;
				node->depth = 1;
				tree->children->insert(std::pair<unsigned int, Node*>(s, node));
			}
		}

		virtual ~FreqSubt()
		{
			delete_tree(tree);
		}

		const unsigned int train(const std::string training_traj_file_name);

		const unsigned int test(const std::string test_traj_file_name);

		void add_trajectory(Trajectory* traj);
		
		void add_trajectory
			(
				Trajectory::iterator it, 
				Trajectory* traj, 
				Node* tree,
				const unsigned int depth,
				std::list<Node*>* suffix_pointers,
				std::list<Node*>* new_suffix_pointers
			);

		void freq_sub_traj(std::list<Trajectory*>& fsts);

		void print();
		
		CompTrajectory* compress(Trajectory* traj) const;
	private:
		double min_sup;
		unsigned int max_length;
		Node* tree;
		unsigned int size_tree;
		unsigned int id;
		
		void print_tree(Node* node);
		void print_tree(Node* node, const std::string str);
		void delete_tree(Node* node);
		void freq_sub_traj(std::list<Trajectory*>& fsts, Node* node, Trajectory* traj=NULL);
		void prune_unfrequent_subtraj();
		void prune_tree(Node* root);
		Node* new_node();
};

class CompTrajDBPostGis:public TrajDBPostGis
{
	public:
		CompTrajDBPostGis():TrajDBPostGis(){}
		virtual ~CompTrajDBPostGis(){};
	protected:
		 static const std::string table_name;

};

class FreqSubtCompTrajDB:public TrajDB
{
	public:
		FreqSubtCompTrajDB
			(
				const std::string& train_file,
				const double _min_sup,
				const double _max_length,
				RoadNet* _net
			):TrajDB(_net)
		{
			alg = new FreqSubt(_min_sup, _max_length, _net);
			alg->train(train_file);
		}

		virtual ~FreqSubtCompTrajDB()
		{
			delete alg;
		}
		
		const bool insert(const std::string& obj, Trajectory& traj);
		
		const bool insert(const std::string& obj, CompTrajectory& traj);

		const bool insert
			(
				const std::string& obj,
				const seg_time& st
			);

		const bool center_radius_query
			(
				const unsigned int lat,
				const unsigned int longit,
				const double dist,
				std::list<std::string>& res,
				const unsigned int time_begin,
				const unsigned int time_end
			)
				const;
	private:
		TrajCompAlgo* alg;
};
#endif

