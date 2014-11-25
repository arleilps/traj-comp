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

const std::string CompTrajDBPostGis::table_name = "comptraj";

Node* FreqSubt::new_node()
{
	Node* node = new Node;
	node->id = id++;
	node->freq = 0;
	node->seg = 0;
	node->depth = 0;
	node->children = new std::map<unsigned int, Node*>;
 	node->suffix = NULL; 
	
	return node;
}

TrajCompAlgo::TrajCompAlgo(RoadNet* _net)
{
	net = _net;
}

TrajCompAlgo::~TrajCompAlgo()
{
}

const unsigned int FreqSubt::train(const std::string training_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory::read_trajectories(trajectories, training_traj_file_name, net);
	
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		add_trajectory(*it);

		delete *it;
	}

	prune_unfrequent_subtraj();

//	print_tree(tree);

	return size_tree;
}

const unsigned int FreqSubt::test(const std::string test_traj_file_name)
{
/*	
	Trajectory* traj = new Trajectory();
	traj->add_update(0,0);
	traj->add_update(3,0);
	traj->add_update(6,0);
	traj->add_update(4,0);
	traj->add_update(7,0);
	traj->add_update(5,0);
	traj->add_update(2,0);
	traj->add_update(0,0);
	traj->add_update(4,0);
	traj->add_update(1,0);
	traj->add_update(9,0);
	std::list<Node*> compressed;
	
	std::cout << "size = " << compress(traj, compressed) << std::endl;;
*/
	std::list<Trajectory*> trajectories;
	unsigned int updates = 0;
	Trajectory* traj;
	CompTrajectory* comp_traj;
	std::list<Node*> compressed;

	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);

	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		comp_traj = compress(traj);
		updates += comp_traj->size();
		delete comp_traj;
	}

	return updates;
}

void FreqSubt::add_trajectory
	(
		Trajectory::iterator it, 
		Trajectory* traj, 
		Node* tree, 
		const unsigned int depth,
		std::list<Node*>* suffix_pointers,
		std::list<Node*>* new_suffix_pointers
	)
{
	tree->freq++;
	
	if(it != traj->end() && depth < max_length)
	{
		std::map<unsigned int, Node*>::iterator node = tree->children->find((*it)->segment);
		
		if(node != tree->children->end())
		{
			suffix_pointers->pop_front();
			new_suffix_pointers->push_back(node->second);
			
			add_trajectory
				(
					++it, 
					traj, 
					node->second, 
					depth+1, 
					suffix_pointers, 
					new_suffix_pointers
				);
		}
		else
		{
			size_tree++;
			tree->children->insert(std::pair<unsigned int, Node*>((*it)->segment, new_node()));
			id++;
			tree->children->at((*it)->segment)->seg = (*it)->segment;
			tree->children->at((*it)->segment)->depth = depth+1;
			tree->children->at((*it)->segment)->suffix = suffix_pointers->front();
			suffix_pointers->pop_front();
			new_suffix_pointers->push_back(tree->children->at((*it)->segment));
			
			add_trajectory
				(
					++it, 
					traj, 
					tree->children->at((*it)->segment), 
					depth+1, 
					suffix_pointers, new_suffix_pointers
				);
		}
	}
}

void FreqSubt::add_trajectory(Trajectory* traj)
{
	Trajectory::iterator itj;
	std::list<Node*>* suffix_pointers = new std::list<Node*>;
	std::list<Node*>* new_suffix_pointers;
	
	suffix_pointers->push_back(NULL);

	Trajectory::iterator iti = traj->end();
	--iti;

	while(true)
	{
		itj = iti;
		new_suffix_pointers = new std::list<Node*>;
		new_suffix_pointers->push_back(NULL);
		add_trajectory(itj, traj, tree, 0, suffix_pointers, new_suffix_pointers);
		delete suffix_pointers;
		suffix_pointers = new_suffix_pointers;
		
		if(iti == traj->begin())
		{
			break;
		}

		--iti;
	}

	delete suffix_pointers;
}

CompTrajectory* FreqSubt::compress(Trajectory* traj) const
{
	CompTrajectory* comp_traj = new CompTrajectory();
	Node* curr_node = tree;
	std::map<unsigned int, Node*>::iterator node_it;
	std::list<Node*> decomp;
	std::list<unsigned int> start_times;
	std::list<unsigned int> end_times;
	unsigned int size_dec = 0;

	Trajectory::iterator it = traj->begin();

	if(traj->size())
	{
		start_times.push_back((*it)->start_time);
	}
	
	while(it != traj->end())
	{
		node_it = curr_node->children->find((*it)->segment);

		if(node_it == curr_node->children->end())
		{
			if(curr_node->suffix != NULL)
			{
				curr_node = curr_node->suffix;
			}
			else
			{
				curr_node = tree;
			}
		}
		else
		{
			decomp.push_back(node_it->second);
			curr_node = node_it->second;
			++it;
			size_dec++;
		}
	}

	unsigned int l = 0;

	while(size_dec > 0)
	{
		curr_node = decomp.front();
		decomp.pop_front();
		size_dec--;

		if(l == 0)
		{
			comp_traj->add_update(curr_node->id, 0, 0);
			l =  curr_node->depth - 1;
		}
		else
		{
			l--;
		}
	}

	return comp_traj;
}

void FreqSubt::delete_tree(Node* node)
{
	for(std::map<unsigned int, Node*>::iterator it = node->children->begin(); 
		it != node->children->end(); ++it)
	{
		delete_tree(it->second);
	}

	delete node->children;
	delete node;
	size_tree--;
}

void FreqSubt::prune_tree(Node* root)
{
	Node* node;
	
	std::map<unsigned int, Node*>* new_map = new std::map<unsigned int, Node*>; 

	std::map<unsigned int, Node*>::iterator it = root->children->begin(); 
	 
	while(it != root->children->end())
	{
		node = it->second;

		if(node->freq < min_sup)
		{
			delete_tree(node);
		}
		else
		{
			new_map->insert(std::pair<unsigned int, Node*>(it->first, node));
			prune_tree(node);
		}
		
		++it;
	}

	delete root->children;
	root->children = new_map;
}

void FreqSubt::prune_unfrequent_subtraj()
{
	Node* node;

	for(std::map<unsigned int, Node*>::iterator it = tree->children->begin();
		it != tree->children->end(); ++it)
	{
		node = it->second;
		prune_tree(node);
	}
	
}

void FreqSubt::print_tree(Node* node)
{
	std::string str;
	
	for(std::map<unsigned int, Node*>::iterator it = node->children->begin(); 
		it != node->children->end(); ++it)
	{
		str = "[" + to_string(net->seg_name(it->first)) + "]";
		print_tree(it->second, str);
	}
}

void FreqSubt::print_tree(Node* node, const std::string str) 
{
	std::string new_str;
	
	std::cout << str << " " << node->freq << std::endl;

	for(std::map<unsigned int, Node*>::iterator it = node->children->begin(); 
		it != node->children->end(); ++it)
	{
		new_str = str +"-["+ to_string(net->seg_name(it->first)) + "]";
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

		traj->add_update(node->seg, 0, 0);
		fsts.push_back(traj);

		Trajectory* new_traj;

		for(std::map<unsigned int, Node*>::iterator it = node->children->begin(); 
			it != node->children->end(); ++it)
		{
			new_traj = new Trajectory(*traj);
			freq_sub_traj(fsts, it->second, new_traj);
		}
	}
}

void FreqSubt::freq_sub_traj(std::list<Trajectory*>& fsts)
{
	for(std::map<unsigned int, Node*>::iterator it = tree->children->begin(); 
		it != tree->children->end(); ++it)
	{
		freq_sub_traj(fsts, it->second);
	}
}

void FreqSubt::print()
{
	print_tree(tree);
}

const bool FreqSubtCompTrajDB::insert
	(       
		const std::string& obj,
		const seg_time& st
	)
{
	return false;
}

const bool FreqSubtCompTrajDB::insert(const std::string& obj, Trajectory& traj)
{
	CompTrajectory* comp_traj = compress(&traj);

}

const bool FreqSubtCompTrajDB::insert(const std::string& obj, CompTrajectory& traj)
{
	return false;
}

const bool FreqSubtCompTrajDB::center_radius_query
	(
		const unsigned int lat,
		const unsigned int longit,
		const double dist,
		std::list<std::string>& res,
		const unsigned int time_begin=0,
		const unsigned int time_end=0
	)
		const
 {
 	return false;
 }
