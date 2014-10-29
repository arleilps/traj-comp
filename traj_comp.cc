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
 *	FILE traj_comp.cc: Trajectory compression algorithms
**/

/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <iostream>

/*my includes*/
#include "traj_comp.h"
#include "io.h"

TrajCompAlgo::TrajCompAlgo(RoadNet* _net)
{
	net = _net;
}

const unsigned int TrajCompAlgo::test(const std::string test_traj_file_name)
{
	return 0;
}

TrajCompAlgo::~TrajCompAlgo()
{
}

void FreqSubt::train(const std::string training_traj_file_name)
{
	//TODO
	std::list<Trajectory*>* trajectories = Trajectory::read_trajectories(training_traj_file_name, net);
	
	for(std::list<Trajectory*>::iterator it = trajectories->begin();
		it != trajectories->end(); ++it)
	{
		add_trajectory(*it);
	}

	Trajectory::delete_trajectories(trajectories);

	print_tree(tree);
}

const unsigned int FreqSubt::test(const std::string test_traj_file_name)
{
	//TODO
	return 0;
}

void FreqSubt::add_trajectory(Trajectory::iterator it, Trajectory* traj, Node* tree)
{
	tree->freq++;
	
	if(it != traj->end())
	{
		std::map<unsigned int, Node*>::iterator node = tree->children.find((*it)->segment);
		
		if(node != tree->children.end())
		{
			add_trajectory(++it, traj, node->second);
		}
		else
		{
			tree->children[(*it)->segment] = new Node;
			tree->children[(*it)->segment]->freq = 0;
			tree->children[(*it)->segment]->seg = (*it)->segment;
			add_trajectory(++it, traj, tree->children[(*it)->segment]);
		}
	}
}

void FreqSubt::add_trajectory(Trajectory* traj)
{
	Trajectory::iterator itj;
	
	for(Trajectory::iterator iti = traj->begin();
		iti != traj->end(); ++iti)
	{
		itj = iti;
		add_trajectory(itj, traj, tree);
	}
}

void FreqSubt::delete_tree(Node* node)
{
	for(std::map<unsigned int, Node*>::iterator it = node->children.begin(); 
		it != node->children.end(); ++it)
	{
		delete_tree(it->second);
	}

	delete node;
}

void FreqSubt::print_tree(Node* node)
{
	std::string str;
	
	for(std::map<unsigned int, Node*>::iterator it = node->children.begin(); 
		it != node->children.end(); ++it)
	{
		str = to_string(it->first);
		print_tree(it->second, str);
	}
}

void FreqSubt::print_tree(Node* node, const std::string str) 
{
	std::string new_str;
	
	std::cout << str << " " << node->freq << std::endl;

	for(std::map<unsigned int, Node*>::iterator it = node->children.begin(); 
		it != node->children.end(); ++it)
	{
		new_str = str + to_string(it->first);
		print_tree(it->second, new_str);
	}
}

void FreqSubt::freq_sub_traj(std::list<Trajectory*>& fsts, Node* node, Trajectory* traj)
{
	if(node->freq >= min_sup)
	{
		if(traj == NULL)
		{
			traj = new Trajectory();
		}

		traj->add_update(node->seg, 0);
		fsts.push_back(traj);

		Trajectory* new_traj;

		for(std::map<unsigned int, Node*>::iterator it = node->children.begin(); 
			it != node->children.end(); ++it)
		{
			new_traj = new Trajectory(*traj);
			freq_sub_traj(fsts, it->second, new_traj);
		}
	}
}

void FreqSubt::freq_sub_traj(std::list<Trajectory*>& fsts)
{
	for(std::map<unsigned int, Node*>::iterator it = tree->children.begin(); 
		it != tree->children.end(); ++it)
	{
		freq_sub_traj(fsts, it->second);
	}
}

void FreqSubt::print()
{
	print_tree(tree);
}
