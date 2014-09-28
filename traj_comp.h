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

typedef struct t_node
{
	unsigned int seg;
	unsigned int freq;
	std::map<unsigned int, t_node*> children;
}Node;

/**
 * Generic trajectory compression algorithm
**/
class TrajCompAlgo
{
	public:
		TrajCompAlgo(RoadNet* _net);
		virtual void train(const std::string training_traj_file_name){};
		virtual const unsigned int test(const std::string test_traj_file_name);
		virtual ~TrajCompAlgo();
		virtual void update(const unsigned int& obj, const unsigned int& seg, 
			const unsigned int timestamp){};
	protected:
		RoadNet* net;
};

class FreqSubt: public TrajCompAlgo
{
	public:
		FreqSubt(const double _min_sup, RoadNet* net)
			:TrajCompAlgo(net)
		{
			min_sup = _min_sup;
			tree = new Node;
			tree->seg = 0;
			tree->freq = 0;
		}

		virtual ~FreqSubt()
		{
			delete tree;
		}

		void train(const std::string training_traj_file_name);

		const unsigned int test(const std::string test_traj_file_name);

	private:
		double min_sup;
		Node* tree;
		
		void add_trajectory(Trajectory* traj);
		void add_trajectory(Trajectory::iterator it, Trajectory* traj, Node* tree);
		void print_tree(Node* node);
		void print_tree(Node* node, const std::string str);
		void delete_tree(Node* node);
};
#endif

