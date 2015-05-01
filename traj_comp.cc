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
#include <math.h>

#include <iostream>
#include <cassert>

#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Quotient.h>

typedef CGAL::MP_Float ET;
typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

/*my includes*/
#include "traj_comp.h"
#include "io.h"

node_subt* FreqSubt::new_node()
{
	node_subt* node = new node_subt;
	node->id = id++;
	node->freq = 0;
	node->seg = 0;
	node->depth = 0;
	node->children = new std::map<unsigned int, node_subt*>;
 	node->suffix = NULL; 
	
	return node;
}

TrajCompAlgo::TrajCompAlgo(RoadNet* _net)
{
	net = _net;
	
	_compression_time = 0;
	_training_time = 0;
	_num_updates_orig = 0;
	_num_updates_comp = 0;
	_num_updates_train = 0;
	_num_traj_comp = 0;
	_num_traj_train = 0;
	comp_t = new ExecTime();
	train_t = new ExecTime();
}

TrajCompAlgo::~TrajCompAlgo()
{
	delete comp_t;
	delete train_t;
}

//Frequent subtrajectories are identified in the training file by building
//a frequent subtrajectory tree, which is similar to a frequent substring tree
void FreqSubt::train(const std::string training_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory::read_trajectories(trajectories, training_traj_file_name, net);
	
	train_t->start();
	
	//Adds each trajectory into the tree
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		_num_traj_train++;
		_num_updates_train += (*it)->size();
		
		add_trajectory(*it);

		delete *it;
	}

	//Prunes unfrequent subtrajectories based on the given minimum support
	prune_unfrequent_subtraj();

	//Sets an index for the frequent subtrajectories that contain a given segment
	set_seg_index();
	
	train_t->stop();
	_training_time = train_t->get_seconds();
}

void FreqSubt::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	CompTrajectory* comp_traj;
	std::list<Trajectory*> decomp;

	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);

	//Compresses each trajectory in the file and computes the total
	//number of updates
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		traj->decompose_delay(decomp, delay, net);

		for(std::list<Trajectory*>::iterator jt = decomp.begin();
			jt != decomp.end(); ++jt)
		{
			comp_traj = compress(*jt);
			delete comp_traj;
			delete *jt;
		}
		
		decomp.clear();
		delete traj;
		_num_traj_comp++;
	}
	
	_compression_time = comp_t->get_seconds();
}

//Each node in the tree has a map from a segment id to a child node
void FreqSubt::add_trajectory
	(
		Trajectory::iterator it, 
		Trajectory* traj, 
		node_subt* tree, 
		const unsigned int depth,
		std::list<node_subt*>* suffix_pointers,
		std::list<node_subt*>* new_suffix_pointers
	)
{
	tree->freq++;
	
	if(it != traj->end() && depth < max_length)
	{
		//Searches for the segment in the map
		std::map<unsigned int, node_subt*>::iterator node = tree->children->find((*it)->segment);
		
		//Creates a new node if the segment is not found
		if(node != tree->children->end())
		{
			suffix_pointers->pop_front();
			new_suffix_pointers->push_back(node->second);
			
			//Recursively adds the remaining (suffix) of the trajectory
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
			//Creating a new node and inserting into the tree
			size_tree++;
			tree->children->insert(std::pair<unsigned int, node_subt*>((*it)->segment, new_node()));
			id++;
			tree->children->at((*it)->segment)->seg = (*it)->segment;
			tree->children->at((*it)->segment)->depth = depth+1;
			tree->children->at((*it)->segment)->suffix = suffix_pointers->front();
			suffix_pointers->pop_front();
			new_suffix_pointers->push_back(tree->children->at((*it)->segment));
			
			//Recursively adds the remaining (suffix) of the trajectory
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
	
	//Each node in the tree has a pointer to its largest suffix
	//which is useful for decomposing a trajectory into 
	//subtrajectories
	std::list<node_subt*>* suffix_pointers = new std::list<node_subt*>;
	std::list<node_subt*>* new_suffix_pointers;
	
	suffix_pointers->push_back(NULL);

	//Trajectory added by iterating over it
	Trajectory::iterator iti = traj->end();
	--iti;
	
	//Doesn't look good.
	while(true)
	{
		itj = iti;
		new_suffix_pointers = new std::list<node_subt*>;
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

CompTrajectory* FreqSubt::compress(Trajectory* traj)
{
	comp_t->start();

	CompTrajectory* comp_traj = new CompTrajectory();
	node_subt* curr_node = tree;
	std::map<unsigned int, node_subt*>::iterator node_it;
	std::list<node_subt*> decomp;
	unsigned int size_dec = 0;

	Trajectory::iterator it = traj->begin();

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
		curr_node = decomp.back();

		decomp.pop_back();
		size_dec--;

		if(l == 0)
		{
			comp_traj->add_update_front(curr_node->id, 0, 0);
			l =  curr_node->depth - 1;
		}
		else
		{
			comp_traj->front()->time = 0;
			l--;
		}
	}
	
	comp_t->stop();
	_num_updates_orig += traj->size();
	_num_updates_comp += comp_traj->size();
	
	return comp_traj;
}

void FreqSubt::delete_tree(node_subt* node)
{
	for(std::map<unsigned int, node_subt*>::iterator it = node->children->begin(); 
		it != node->children->end(); ++it)
	{
		delete_tree(it->second);
	}

	delete node->children;
	delete node;
	size_tree--;
}

void FreqSubt::prune_tree(node_subt* root)
{
	node_subt* node;
	
	std::map<unsigned int, node_subt*>* new_map = new std::map<unsigned int, node_subt*>; 

	std::map<unsigned int, node_subt*>::iterator it = root->children->begin(); 
	 
	while(it != root->children->end())
	{
		node = it->second;

		if(node->freq < min_sup)
		{
			delete_tree(node);
		}
		else
		{
			new_map->insert(std::pair<unsigned int, node_subt*>(it->first, node));
			prune_tree(node);
		}
		
		++it;
	}

	delete root->children;
	root->children = new_map;
}

void FreqSubt::prune_unfrequent_subtraj()
{
	node_subt* node;

	for(std::map<unsigned int, node_subt*>::iterator it = tree->children->begin();
		it != tree->children->end(); ++it)
	{
		node = it->second;
		prune_tree(node);
	}
}

void FreqSubt::set_seg_index(node_subt* root)
{
	node_subt* node;
	
	for(std::map<unsigned int, node_subt*>::iterator it = root->children->begin(); 
		it != root->children->end(); ++it)
	{
		node = it->second;
		seg_to_freq_subt_index.at(node->seg)->push_back(node->id);
		set_seg_index(node);
	}
}

void FreqSubt::set_seg_index()
{
	node_subt* node;

	for(std::map<unsigned int, node_subt*>::iterator it = tree->children->begin();
		it != tree->children->end(); ++it)
	{
		node = it->second;
		set_seg_index(node);
	}
}

void FreqSubt::print_tree(node_subt* node)
{
	std::string str;
	
	for(std::map<unsigned int, node_subt*>::iterator it = node->children->begin(); 
		it != node->children->end(); ++it)
	{
		str = "[" + to_string(net->seg_name(it->first)) + "]";
		print_tree(it->second, str);
	}
}

void FreqSubt::print_tree(node_subt* node, const std::string str) 
{
	std::string new_str;
	
	std::cout << str << " " << node->id << std::endl;

	for(std::map<unsigned int, node_subt*>::iterator it = node->children->begin(); 
		it != node->children->end(); ++it)
	{
		new_str = str +"-["+ to_string(net->seg_name(it->first)) + "]";
		print_tree(it->second, new_str);
	}
}

void FreqSubt::freq_sub_traj
	(
		std::list<std::pair<unsigned int, Trajectory * > * >& fsts,
		node_subt* node, 
		Trajectory* traj
	)
{
	if(node->freq >= min_sup)
	{
		if(traj == NULL)
		{
			traj = new Trajectory();
		}

		traj->add_update(node->seg, 0, 0);
		
		//Adding trajectory and corresponding node id in the freq subt tree
		//to the list
		fsts.push_back(new std::pair<unsigned int, Trajectory*>(node->id, traj));

		Trajectory* new_traj;

		for(std::map<unsigned int, node_subt*>::iterator it = node->children->begin(); 
			it != node->children->end(); ++it)
		{
			new_traj = new Trajectory(*traj);
			freq_sub_traj(fsts, it->second, new_traj);
		}
	}
}

void FreqSubt::freq_sub_traj
	(
		std::list<std::pair<unsigned int, Trajectory * > * >& fsts
	)
{
	for(std::map<unsigned int, node_subt*>::iterator it = tree->children->begin(); 
		it != tree->children->end(); ++it)
	{
		freq_sub_traj(fsts, it->second);
	}
}

void FreqSubt::print()
{
	print_tree(tree);
}

void FreqSubt::get_freq_subt_ids
	(
		const unsigned int seg, 
		std::list<unsigned int>& freq_subt_ids
	)
		const
{
	freq_subt_ids.clear();
	for(std::list<unsigned int>::iterator it = seg_to_freq_subt_index.at(seg)->begin();
		it != seg_to_freq_subt_index.at(seg)->end(); ++it)
	{
		freq_subt_ids.push_back(*it);
	}
}

/*
const bool FreqSubtCompTrajDB::insert
	(       
		const std::string& obj,
		const seg_time& st
	)
{
	return false;
}

const bool FreqSubtCompTrajDB::insert(const std::string& input_file_name)
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

const bool FreqSubtCompTrajDB::insert(const std::string& obj, Trajectory& traj)
{
	CompTrajectory* comp_traj = alg->compress(&traj);
	n_updates += traj.size();

//	std::cout << "traj-size = " << traj.size() << " comptraj-size = " 
//		<< comp_traj->size() << std::endl;

	bool status;

	for(Trajectory::iterator it = comp_traj->begin();
		it != comp_traj->end(); ++it)
	{
		if(! db->insert(obj, *(*it)))
		{
			status = false;
		}
	}

	return status;
}

//TODO: Maybe there is a way to index the hypersegments
//in the database directly, instead of recovering them
//from segments. Check multilinestring.
const bool FreqSubtCompTrajDB::center_radius_query
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
	bool status = net->segments_within_distance(segs, latit, longit, dist);
	std::list<unsigned int> freq_subt_ids;
	std::list<unsigned int> freq_subt_ids_seg;
	std::list<std::string> objs;

	res.clear();

	for(std::list<unsigned int>::iterator it = segs.begin();
		it != segs.end(); ++it)
	{
		alg->get_freq_subt_ids(*it, freq_subt_ids_seg);
		
		for(std::list<unsigned int>::iterator it_subt = freq_subt_ids_seg.begin();
			it_subt != freq_subt_ids_seg.end(); ++it_subt)
		{
			freq_subt_ids.push_back(*it_subt);
		}

		freq_subt_ids_seg.clear();
	}

	freq_subt_ids.sort();
	freq_subt_ids.unique();

	for(std::list<unsigned int>::iterator it_subt = freq_subt_ids.begin();
		it_subt != freq_subt_ids.end(); ++it_subt)
	{
		if(! db->query_segment_time(*it_subt, objs, time_begin, time_end))
		{
			status = false;
		}

		res.splice(res.end(), objs);
	}

	res.sort();
	res.unique();

	return status;
}
*/

NodePPM* PredPartMatch::new_node_ppm(const unsigned int segment)
{
	NodePPM* node = new NodePPM;
	
	node->segment = segment;
	node->id = size_tree++;
	node->next = net->neighbors(segment)->front();		
	node->freq_next = 0;

	return node;
}

void PredPartMatch::train(const std::string training_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory::read_trajectories(trajectories, training_traj_file_name, net);

	train_t->start();

	//Adds each trajectory into the tree
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		_num_traj_train++;
		_num_updates_train += (*it)->size();
		
		add_trajectory(*it);

		delete *it;
	}

	train_t->stop();
	_training_time = train_t->get_seconds();
}

void PredPartMatch::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	CompTrajectory* comp_traj;
	
	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);

	//Compresses each trajectory in the file and computes the total
	//number of updates
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		comp_traj = compress(traj);
		delete comp_traj;
		delete traj;
	}
	
	_compression_time = comp_t->get_seconds();
}

void PredPartMatch::add_trajectory
	(
		const Trajectory::iterator& iti, 
		Trajectory::iterator& itj, 
		Trajectory* traj,
		NodePPM* tree,
		const unsigned int r
	)
{
	if(itj != traj->begin() && r <= order)
	{
		std::map<unsigned int, NodePPM*>::iterator node_it = tree->children.find((*itj)->segment);
		NodePPM* node;
		std::map<unsigned int, double>::iterator next;

		if(node_it != tree->children.end())
		{
			node = node_it->second;
			next = node->freq.find((*iti)->segment);

			if(next != node->freq.end())
			{
				next->second++;
				
				if(next->second > node->freq_next)
				{
					node->freq_next = next->second;
					node->next = (*iti)->segment;
				}
			}
			else
			{
				node->freq.insert(std::pair<unsigned int, double>((*iti)->segment, 1));

				if(node->freq.size() == 1)
				{
					node->freq_next = 1;
					node->next = (*iti)->segment;
				}
			}

			add_trajectory(iti, --itj, traj, node, r+1);
		}
		else
		{
			node = new_node_ppm((*itj)->segment);
			tree->children.insert(std::pair<unsigned int, NodePPM*>((*itj)->segment, node));
			node->freq.insert(std::pair<unsigned int, double>((*iti)->segment, 1));
			node->freq_next = 1;
			node->next = (*iti)->segment;

			add_trajectory(iti, --itj, traj, node, r+1);
		}
	}
}

void PredPartMatch::add_trajectory(Trajectory* traj)
{
	Trajectory::iterator itj;
	
	//Trajectory added by iterating over it
	Trajectory::iterator iti = traj->begin();
	iti++;

	while(iti != traj->end())
	{
		itj = iti;
		--itj;
		add_trajectory(iti, itj, traj, tree, 1);
		++iti;
	}
}

const unsigned int PredPartMatch::next_segment
	(
		Trajectory::iterator it, 
		Trajectory* traj,
		NodePPM* tree
	) const
{
	std::map<unsigned int, NodePPM*>::iterator node_it = tree->children.find((*it)->segment);
	NodePPM* node;

	if(node_it != tree->children.end() && it != traj->begin())
	{
		node = node_it->second;

		return next_segment(--it, traj, node);
	}
	else
	{
		return tree->next;
	}
}

CompTrajectory* PredPartMatch::compress(Trajectory* traj)
{
	comp_t->start();
	
	CompTrajectory* comp_traj = new CompTrajectory();
	
	Trajectory::iterator it = traj->begin();
	comp_traj->add_update((*it)->segment, (*it)->time, (*it)->dist);
	unsigned int next;

	while(it != traj->end())
	{
		next = next_segment(it, traj, tree);
		it++;
		
		if(it != traj->end() && (*it)->segment != next)
		{
			comp_traj->add_update
				(
					(*it)->segment, 
					(*it)->time, 
					(*it)->dist
				);
		}
	}
	
	comp_t->stop();
	_num_updates_orig += traj->size();
	_num_updates_comp += comp_traj->size();
	_num_traj_comp++;

	return comp_traj;
}

void PredPartMatch::delete_tree(NodePPM* tree)
{
	std::map<unsigned int, NodePPM*>::iterator node_it;
	NodePPM* node;

	for(node_it = tree->children.begin(); 
		node_it != tree->children.end();
		++node_it)
	{
		node = node_it->second;

		delete_tree(node);
	}

	delete tree;
}

void ShortestPath::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	CompTrajectory* comp_traj;
	std::list<Trajectory*> decomp;

	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);

	//Compresses each trajectory in the file and computes the total
	//number of updates
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		traj->decompose_delay(decomp, delay, net);

		for(std::list<Trajectory*>::iterator jt = decomp.begin();
			jt != decomp.end(); ++jt)
		{
			comp_traj = compress(*jt);
			delete comp_traj;
			delete *jt;
		}
		
		decomp.clear();
		delete traj;
		_num_traj_comp++;
	}
	
	_compression_time = comp_t->get_seconds();
}

bool ShortestPath::check_sp_through
	(
		const unsigned int start, 
		const unsigned int end, 
		const unsigned int through
	) const
{
	std::map<unsigned int, unsigned int>::iterator it;

	it = short_paths.at(start)->find(end);

	if(it != short_paths.at(start)->end())
	{
		if(it->second == through)
		{
			return true;
		}
	}
	
	return false;
}

CompTrajectory* ShortestPath::compress(Trajectory* traj) 
{
	comp_t->start();
	CompTrajectory* comp_traj = new CompTrajectory();
		
	Trajectory::iterator it = traj->begin();
	unsigned int start = (*it)->segment;
		
	comp_traj->add_update(start, (*it)->time, (*it)->dist);
		
	++it;

	if(it != traj->end())
	{
		unsigned int b_end = (*it)->segment;
		unsigned int end;
		++it;

		while(it != traj->end())
		{
			end = (*it)->segment;
				
			if(! check_sp_through(start, end, b_end))	
			{
				comp_traj->add_update(b_end, 0, 0);
				start = b_end;
			}

			b_end = (*it)->segment;
			++it;
		}

		comp_traj->add_update(b_end, 0, 0);
	}
	
	comp_t->stop();
	_num_updates_orig += traj->size();
	_num_updates_comp += comp_traj->size();

	return comp_traj;
}

void ShortestPath::compute_shortest_paths()
{
	net->fill_short_path_struct(max_length, short_paths, num_threads);
}

void ShortestPath::delete_shortest_paths()
{
	for(unsigned int s = 0; s < net->size(); s++)
	{
		delete short_paths.at(s);
	}
}

void ShortestPathFreqSubt::train(const std::string training_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory::read_trajectories(trajectories, training_traj_file_name, net);
	
	train_t->start();

	CompTrajectory* sp_comp;
	//Adds each trajectory into the tree
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		_num_traj_train++;
		_num_updates_train += (*it)->size();
		
		sp_comp = shortest_path_comp->compress(*it);
		freq_subt_comp->add_trajectory(sp_comp);

		delete sp_comp;
		delete *it;
	}

	//Prunes unfrequent subtrajectories based on the given minimum support
	freq_subt_comp->prune_unfrequent_subtraj();

	//Sets an index for the frequent subtrajectories that contain a given segment
	freq_subt_comp->set_seg_index();

	train_t->stop();
	_training_time = train_t->get_seconds();
}

void ShortestPathFreqSubt::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	CompTrajectory* comp_traj;
	std::list<Trajectory*> decomp;

	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);

	//Compresses each trajectory in the file and computes the total
	//number of updates
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		traj->decompose_delay(decomp, delay, net);

		for(std::list<Trajectory*>::iterator jt = decomp.begin();
			jt != decomp.end(); ++jt)
		{
			comp_traj = compress(*jt);
			delete comp_traj;
			delete *jt;
		}
		
		decomp.clear();
		delete traj;
		_num_traj_comp++;
	}
	
	_compression_time = comp_t->get_seconds();
}
	
CompTrajectory* ShortestPathFreqSubt::compress(Trajectory* traj)
{
	comp_t->start();

	CompTrajectory* sp_comp;
	CompTrajectory* fs_comp;
	
	sp_comp = shortest_path_comp->compress(traj);
	fs_comp = freq_subt_comp->compress(sp_comp);
	
	delete sp_comp;
	
	comp_t->stop();
	_num_updates_orig += traj->size();
	_num_updates_comp += fs_comp->size();
	
	return fs_comp;
}

void TSND::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	std::list < dist_time* > dist_times;
	std::list < dist_time* > comp_dist_times;
	
	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);

	//Compresses each trajectory in the file and computes the total
	//number of updates
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;

		traj->get_dist_times_uniform(dist_times, net);

		compress(dist_times, comp_dist_times);

		Trajectory::delete_dist_times(dist_times);
		Trajectory::delete_dist_times(comp_dist_times);
		delete traj;
	}
	
	_compression_time = comp_t->get_seconds();
}

void NSTD::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	std::list < dist_time* > dist_times;
	std::list < dist_time* > comp_dist_times;
	
	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);

	//Compresses each trajectory in the file and computes the total
	//number of updates
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;

		traj->get_dist_times_uniform(dist_times, net);

		compress(dist_times, comp_dist_times, *traj);

		Trajectory::delete_dist_times(dist_times);
		Trajectory::delete_dist_times(comp_dist_times);
		delete traj;
	}
	
	_compression_time = comp_t->get_seconds();
}

void TSND::compress
	(
		std::list < dist_time* >& dist_times,
		std::list < dist_time* >& comp_dist_times
	)
{
	comp_dist_times.clear();
	comp_t->start();
	dist_time* p_i;
	dist_time* p_i_minus_one;
	dist_time* p_index;
	angle R;

	p_i = new_dist_time(dist_times.front()->dist, dist_times.front()->time);
	p_index = dist_times.front();

	comp_dist_times.push_back(p_i);

	std::list < dist_time* >::iterator it = dist_times.begin();
	p_i_minus_one = *it;
	++it;
	R.from = (double) -PI / 2;
	R.to = (double) PI / 2;

	while(it != dist_times.end())
	{
		p_i = *it;

		if(fall_inside(R, *p_index, *p_i))
		{
			constrain(R, *p_index, *p_i, max_error);
		}
		else
		{
			p_index = new_dist_time(p_i_minus_one->dist, p_i_minus_one->time);
			comp_dist_times.push_back(p_index);

			R.from = (double) -PI / 2;
			R.to = (double) PI / 2;
		}
		
		p_i_minus_one = p_i;
		++it;
	}
			
	p_index = new_dist_time(dist_times.back()->dist, dist_times.back()->time);
	comp_dist_times.push_back(p_index);
	
	comp_t->stop();
	_num_updates_orig += dist_times.size();
	_num_updates_comp += comp_dist_times.size();
	_num_traj_comp++;
}

void NSTD::compress
	(
		std::list < dist_time* >& dist_times,
		std::list < dist_time* >& comp_dist_times,
		Trajectory& traj
	)
{
	comp_dist_times.clear();
	comp_t->start();
	dist_time* p_i;
	dist_time* p_i_minus_one;
	dist_time* p_index;
	angle R;

	p_i = new_dist_time(dist_times.front()->dist, dist_times.front()->time);
	p_index = dist_times.front();

	comp_dist_times.push_back(p_i);

	std::list < dist_time* >::iterator it = dist_times.begin();
	p_i_minus_one = *it;
	++it;
	R.from = (double) -PI / 2;
	R.to = (double) PI / 2;

	while(it != dist_times.end())
	{
		p_i = *it;

		if(fall_inside(R, *p_index, *p_i) 
			&& p_i_minus_one->time - comp_dist_times.back()->time < max_error)
		{
			constrain(R, *p_index, *p_i, max_error);
		}
		else
		{
			p_index = new_dist_time(p_i_minus_one->dist, p_i_minus_one->time);
			comp_dist_times.push_back(p_index);

			R.from = (double) -PI / 2;
			R.to = (double) PI / 2;
			constrain(R, *p_index, *p_i, max_error);
		}
		
		p_i_minus_one = p_i;
		++it;
	}
			
	p_index = new_dist_time(dist_times.back()->dist, dist_times.back()->time);
	comp_dist_times.push_back(p_index);
	
	comp_t->stop();
	_num_updates_orig += traj.size();
	_num_updates_comp += comp_dist_times.size();
	_num_traj_comp++;
}

double equal_double(const double d1, const double d2)
{
	return (fabs(d1 - d2) <= 0.001);
}

bool TSND::fall_inside
	(
		const angle& R, 
		const dist_time& p_index,
		const dist_time& p_i	
	)
{
	double tan_from;
	double tan_to;
	double b_from;
	double b_to;

	tan_from = tan(R.from);
	tan_to = tan(R.to);
	
	if(equal_double(fabs(R.from), (double) PI / 2.))
	{
		b_from = -std::numeric_limits<double>::max();
		tan_from = 0;
	}
	else
	{
		if(equal_double(tan_from, 0.0))
		{
			b_from = p_index.dist;
		}
		else
		{
			b_from = (double) p_index.dist - ((double) tan_from * p_index.time);
		}
	}
			
	if(equal_double(fabs(R.to), (double) PI / 2.))
	{
		b_to = std::numeric_limits<double>::max();
		tan_to = 0;
	}
	else
	{
		if(equal_double(tan_to, 0.0))
		{
			b_to = p_index.dist;
		}
		else
		{
			b_to = (double) p_index.dist - ((double) tan_to * p_index.time);
		}
	}

	if((p_i.dist <= (double) tan_to * p_i.time + b_to
		&& p_i.dist >= (double) tan_from * p_i.time + b_from)
		|| (equal_double(p_i.dist, (double) tan_to * p_i.time + b_to)
		&& equal_double(p_i.dist, (double) tan_from * p_i.time + b_from)))
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool NSTD::fall_inside
	(
		const angle& R, 
		const dist_time& p_index,
		const dist_time& p_i	
	)
{
	double tan_from;
	double tan_to;
	double b_from;
	double b_to;

	tan_from = tan(R.from);
	tan_to = tan(R.to);
	
	if(equal_double(fabs(R.from), (double) PI / 2.))
	{
		b_from = -std::numeric_limits<double>::max();
		tan_from = 0;
	}
	else
	{
		if(equal_double(tan_from, 0.0))
		{
			b_from = p_index.dist;
		}
		else
		{
			b_from = (double) p_index.dist - ((double) tan_from * p_index.time);
		}
	}
			
	if(equal_double(fabs(R.to), (double) PI / 2.))
	{
		b_to = std::numeric_limits<double>::max();
		tan_to = 0;
	}
	else
	{
		if(equal_double(tan_to, 0.0))
		{
			b_to = p_index.dist;
		}
		else
		{
			b_to = (double) p_index.dist - ((double) tan_to * p_index.time);
		}
	}

	if(p_i.dist <= (double) tan_to * p_i.time + b_to
		&& p_i.dist >= (double) tan_from * p_i.time + b_from)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void TSND::constrain
	(
		angle& R, 
		const dist_time& p_index,
		const dist_time& p_i,
		const double error
	)
{
	double tan_from;
	double tan_to;
	double b_from;
	double b_to;

	tan_from = tan(R.from);
	tan_to = tan(R.to);

	if(equal_double(fabs(R.from), (double) PI / 2.))
	{
		b_from = -std::numeric_limits<double>::max();
		tan_from = 0;
	}
	else
	{
		if(equal_double(tan_from, 0.0))
		{
			b_from = p_index.dist;
		}
		else
		{
			b_from = (double) p_index.dist - ((double) tan_from * p_index.time);
		}
	}
			
	if(equal_double(R.to, (double) PI / 2.))
	{
		b_to = std::numeric_limits<double>::max();
		tan_to = 0;
	}
	else
	{
		if(equal_double(tan_to, 0))
		{
			b_to = p_index.dist;
		}
		else
		{
			b_to = (double) p_index.dist - ((double) tan_to * p_index.time);
		}
	}

	if(p_i.dist + error < tan_to * p_i.time + b_to)
	{
		if(equal_double(p_i.time, p_index.time)) 
		{
			R.to = (double) PI / 2;
		}
		else
		{
			R.to = atan2((p_i.dist + error - p_index.dist), (p_i.time - p_index.time));
		}
		
	}

	if(p_i.dist - error > tan_from * p_i.time + b_from)
	{
		if(equal_double(p_i.time, p_index.time)) 
		{
			R.from = (double) PI / 2;
		}
		else
		{
			tan_from = 
				(double) (p_i.dist - error - p_index.dist) / (p_i.time - p_index.time);
			R.from = atan2((p_i.dist - error - p_index.dist), (p_i.time - p_index.time));
		}
	}
}

void NSTD::constrain
	(
		angle& R, 
		const dist_time& p_index,
		const dist_time& p_i,
		const double error
	)
{
	double tan_from;
	double tan_to;
	double b_from;
	double b_to;
	double x_from;
	double x_to;

	tan_from = tan(R.from);
	tan_to = tan(R.to);

	if(equal_double(R.from, (double) -PI / 2.))
	{
		x_from = std::numeric_limits<double>::max();
	}
	else
	{
		if(equal_double(tan_from, 0.0))
		{
			x_from = std::numeric_limits<double>::max();
		}
		else
		{
			b_from = (double) p_index.dist - ((double) tan_from * p_index.time);
			x_from = (double) (p_i.dist - b_from) / tan_from;
		}
	}
			
	if(equal_double(fabs(R.to), (double) PI / 2.))
	{
		x_to = p_index.time;
	}
	else
	{
		if(equal_double(tan_to, 0))
		{
			x_to = -std::numeric_limits<double>::max();
		}
		else
		{
			b_to = (double) p_index.dist - ((double) tan_to * p_index.time);
			x_to = (double) (p_i.dist - b_to) / tan_to;
		}
	}

	if(! equal_double(p_i.dist, p_index.dist))
	{
		if(p_i.time - error > x_to)
		{
			if(equal_double(p_i.time - error - p_index.time, 0)) 
			{
				R.to = (double) PI / 2;
			}
			else
			{
				R.to = atan2((p_i.dist - p_index.dist), (p_i.time - error - p_index.time));
			}
		}

		if(p_i.time + error < x_from)
		{
			if(equal_double(p_i.time + error - p_index.time, 0)) 
			{
				R.from = (double) PI / 2;
			}
			else
			{
				R.from = atan2((p_i.dist - p_index.dist), (p_i.time + error - p_index.time));
			}
		}
	}
}

void print_emkf_info
	(
		std::vector < std::vector< std::pair< unsigned int, emkf_update_info* > * > * >& updates_emkf,
		RoadNet* net
	)
{
	std::vector< std::pair< unsigned int, emkf_update_info* > * >* traj;

	for(unsigned int t = 0; t < updates_emkf.size(); t++)
	{
		traj = updates_emkf.at(t);

		for(unsigned int i = 0; i < traj->size(); i++)
		{
			std::cout << "seg: " << net->seg_name(traj->at(i)->first) << std::endl;
			if(traj->at(i)->second != NULL)
			{
				std::cout << "avg speed: " << traj->at(i)->second->avg_speed << std::endl;
				std::cout << "dist: " << traj->at(i)->second->dist << std::endl;
				std::cout << "time: " << traj->at(i)->second->time << std::endl;
				std::cout << "total time: " << traj->at(i)->second->total_time << std::endl;
				std::cout << "sigma: " << traj->at(i)->second->sigma << std::endl;
			}
		}

		std::cout << std::endl;
	}
}

void delete_emkf_info
	(
		std::vector < std::vector< std::pair< unsigned int, emkf_update_info* > * > * >& updates_emkf
	)
{
	std::vector< std::pair< unsigned int, emkf_update_info* > * >* traj;

	for(unsigned int t = 0; t < updates_emkf.size(); t++)
	{
		traj = updates_emkf.at(t);

		for(unsigned int i = 0; i < traj->size(); i++)
		{
			delete traj->at(i)->second;
			delete traj->at(i);
		}

		delete traj;
	}

	updates_emkf.clear();
}

void print_phi(t_phi* phi, t_phi* phi_sigma, RoadNet* net)
{
	std::cout << "PRINT PHI" << std::endl;
	unsigned int s2;
	for(unsigned int s1 = 0; s1 < net->size(); s1++)
	{
		const std::list<unsigned int>* neigh = net->neighbors(s1);

		for(std::list<unsigned int>::const_iterator it = neigh->begin();
			it != neigh->end(); ++it)
		{
			s2 = *it;

			std::cout << "phi(" << net->seg_name(s1) << "," << net->seg_name(s2) << ") = " << phi->at(s1)->at(s2) << std::endl;
			std::cout << "sigma_phi(" << net->seg_name(s1) << "," << net->seg_name(s2) << ") = " << phi_sigma->at(s1)->at(s2) << std::endl;
		}
	}
}

void EMKalman::avg_sigma_speed()
{
	std::vector< std::pair< unsigned int, emkf_update_info* > * >* traj;
	avg_speed = 0;
	sigma_speed = 0;
	sigma_trans = 0;
	unsigned int num = 0;
	double prev_speed;
	unsigned int num_sigma = 0;

	for(unsigned int t = 0; t < updates_emkf.size(); t++)
	{
		traj = updates_emkf.at(t);
		prev_speed = -1;

		for(unsigned int s = 0; s < traj->size(); s++)
		{
			if(traj->at(s)->second != NULL)
			{
				avg_speed += traj->at(s)->second->avg_speed;
				
				if(prev_speed >= 0 &&
					traj->at(s)->second->time > 0 &&
					traj->at(s)->second->avg_speed > 0)
				{
					sigma_trans += pow(
						double(traj->at(s)->second->avg_speed-prev_speed)
						/ traj->at(s)->second->time, 2);
					num_sigma++;
				}

				prev_speed = traj->at(s)->second->avg_speed;

				num++;
			}
		}
	}

	avg_speed = (double) avg_speed / num;
	sigma_trans = sqrt((double) sigma_trans / num_sigma);

	for(unsigned int t = 0; t < updates_emkf.size(); t++)
	{
		traj = updates_emkf.at(t);

		for(unsigned int s = 0; s < traj->size(); s++)
		{
			if(traj->at(s)->second != NULL)
			{
				sigma_speed += pow(avg_speed 
					- traj->at(s)->second->avg_speed, 2);
			}
		}

		sigma_speed = sqrt((double) sigma_speed / num);
	}
}

void EMKalman::train(const std::string training_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	Trajectory::read_trajectories(trajectories, training_traj_file_name, net);
	updates_emkf.reserve(trajectories.size());
	train_t->start();
	
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		_num_traj_train++;
		_num_updates_train += traj->size();
		traj->get_emkf_rep(updates_emkf, sigma_gps, net);
		delete traj;
	}
	
	avg_sigma_speed();
	//print_emkf_info(updates_emkf, net);
	std::pair<t_phi*, t_phi*>* phi_sigma_pair = EM();
	phi = phi_sigma_pair->first;
	phi_sigma = phi_sigma_pair->second;
	//print_phi(phi, phi_sigma, net);
	delete phi_sigma_pair;
	delete_emkf_info(updates_emkf);

	train_t->stop();
	_training_time = train_t->get_seconds();
}

void delete_emkf_comp(std::list<emkf_up*>& emkf_comp)
{
	for(std::list<emkf_up*>::iterator it = emkf_comp.begin();
		it != emkf_comp.end(); ++it)
	{
		delete *it;
	}

	emkf_comp.clear();
}

void EMKalman::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	std::list<emkf_up*> emkf_comp;
	
	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);
	updates_emkf.reserve(trajectories.size());

	//Compresses each trajectory in the file and computes the total
	//number of updates
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		traj->get_emkf_rep(updates_emkf, sigma_gps, net);
		compress(*(updates_emkf.back()), emkf_comp, *traj);
		
		delete_emkf_comp(emkf_comp);
		delete traj;
	}
	
	delete_emkf_info(updates_emkf);
	
	_compression_time = comp_t->get_seconds();
}

void add_update
	(
		std::list<emkf_up*>& emkf_comp, const unsigned int seg, 
		const unsigned int time
	)
{
	emkf_up* e = new emkf_up;

	e->seg = seg;
	e->time = time;

	emkf_comp.push_back(e);
}

void EMKalman::compress
	(
		std::vector< std::pair< unsigned int, emkf_update_info* > * >& traj,
		std::list<emkf_up*>& emkf_comp, Trajectory& trajj
	) 
{
	comp_t->start();
	emkf_update_info* up;
	std::vector<double> prev_speeds;
	std::vector<double> prev_sigmas;
	unsigned int num;
	double avg_up;

	prev_speeds.reserve(traj.size());
	prev_sigmas.reserve(traj.size());
	
	num = 0;
	for(unsigned int s = 0; s < traj.size(); s++)
	{
		if(traj.at(s)->second != NULL)
		{
			up = traj.at(s)->second;
			avg_up = up->avg_speed;

			for(unsigned int i = 0; i < num+1; i++)
			{
				prev_speeds.push_back(avg_up);
				prev_sigmas.push_back(up->time * sigma_trans + SMALLDOUBLE);
			}

			num = 0;
		}
		else
		{
			num++;
		}
	}
	

	add_update(emkf_comp, trajj.front()->segment, trajj.front()->time);
	
	EMKalman::kalman_filter_comp
		(
			traj, prev_speeds, prev_sigmas, *phi, *phi_sigma,
			sigma_trans, emkf_comp
		);

	comp_t->stop();
	_num_updates_orig += traj.size();
	_num_updates_comp += emkf_comp.size();
	_num_traj_comp++;
}


std::pair<t_phi*, t_phi*>*	
	EMKalman::start_phi() const
{
	t_phi* phi_est = new t_phi;
	t_phi* phi_sigma_est = new t_phi;
	unsigned int s2;
	
	for(unsigned int s1 = 0; s1 < net->size(); s1++)
	{
		const std::list<unsigned int>* neigh = net->neighbors(s1);
		
		phi_est->insert
			(
				std::pair<unsigned int, 
					std::map < unsigned int, double > * > 
				(s1, new std::map < unsigned int, double > )
			);

		phi_sigma_est->insert
			(
				std::pair<unsigned int, 
					std::map < unsigned int, double > * > 
				(s1, new std::map < unsigned int, double > )
			);


		for(std::list<unsigned int>::const_iterator it = neigh->begin();
			it != neigh->end(); ++it)
		{
			s2 = *it;

			phi_est->at(s1)->insert(std::pair<unsigned int, double > (s2, 1));
			phi_sigma_est->at(s1)->insert(std::pair<unsigned int, double > (s2, sqrt(sqrt(std::numeric_limits<double>::max()))));
		}
	}

	return (new std::pair<t_phi*, t_phi*>(phi_est, phi_sigma_est));
}

void print_speeds
	(
		std::vector< std::vector< double >* >* speeds, 
		std::vector< std::vector< double >* >* sigmas, 
		const std::vector < std::vector< std::pair< unsigned int, emkf_update_info* > * > * >& updates_emkf, 
		RoadNet* net
	)
{
	std::cout << "PRINT SPEEDS" << std::endl;
	std::vector< std::pair< unsigned int, emkf_update_info* > * >* traj;
	
	for(unsigned int t = 0; t < updates_emkf.size(); t++)
	{
		traj = updates_emkf.at(t);
		for(unsigned int s = 0; s < traj->size(); s++)
		{
			std::cout << "(" << net->seg_name(traj->at(s)->first) << "," << speeds->at(t)->at(s) << "," << sigmas->at(t)->at(s) << ") ";
		}

		std::cout << std::endl;
	}
}

std::pair< std::vector< std::vector< double >* >*, std::vector< std::vector< double > * >* >*
	EMKalman::start_speed() const
{
	std::vector< std::vector< double >* >*
		start_speed_est = new std::vector< std::vector< double >* >;
	std::vector< std::vector< double >* >*
		start_sigma_est = new std::vector< std::vector< double >* >;
	std::vector< std::pair< unsigned int, emkf_update_info* > * >* traj;
	emkf_update_info* up;
	double avg_up;
	unsigned int num;
	start_speed_est->reserve(_num_traj_train);
	start_sigma_est->reserve(_num_traj_train);

	for(unsigned int t = 0; t < updates_emkf.size(); t++)
	{
		traj = updates_emkf.at(t);
		start_speed_est->push_back(new std::vector<double>);
		start_sigma_est->push_back(new std::vector<double>);
		start_speed_est->at(t)->reserve(traj->size());
		start_sigma_est->at(t)->reserve(traj->size());
		
		num = 0;
		for(unsigned int s = 0; s < traj->size(); s++)
		{
			if(traj->at(s)->second != NULL)
			{
				up = traj->at(s)->second;
				avg_up = up->avg_speed;

				for(unsigned int i = 0; i < num+1; i++)
				{
					start_speed_est->at(t)->push_back(avg_up);
					start_sigma_est->at(t)->push_back(up->time * sigma_trans + SMALLDOUBLE);
				}

				num = 0;
			}
			else
			{
				num++;
			}
		}
	}

	return (new std::pair< std::vector< std::vector< double >* >*, std::vector< std::vector< double > * >* >
		(start_speed_est, start_sigma_est));
}

void delete_phi(t_phi* phi)
{
	for(t_phi::iterator it1	= phi->begin(); it1 != phi->end(); ++it1)
	{
		delete it1->second;
	}

	delete phi;
}

void delete_speeds(std::vector< std::vector< double >* >* speeds)
{
	for(unsigned int i = 0; i < speeds->size(); i++)
	{
		delete speeds->at(i);
	}

	delete speeds;
}
		
std::pair<t_phi*, t_phi*>* EMKalman::EM()
{
	std::pair<t_phi*,t_phi*>* phi_sigma = start_phi();
	t_phi* phi_est = phi_sigma->first;
	t_phi* phi_sigma_est = phi_sigma->second;
	delete phi_sigma;
	std::ofstream output_file(output_file_name.c_str(), std::ios::out);

	if(! output_file.is_open())
	{
		std::cerr << "Error: Could not open road network file: "
			<< output_file_name << std::endl << std::endl;
			
		return (new std::pair<t_phi*,t_phi*>(phi_est, phi_sigma_est));
	}

	std::vector< std::vector< double >* >* prev_speeds;
	std::vector< std::vector< double >* >* prev_sigmas;
	std::pair< std::vector< std::vector< double >* >*, std::vector< std::vector< double > * >* >*
		speed_sigma = start_speed();
	
	prev_speeds = speed_sigma->first;
	prev_sigmas = speed_sigma->second;
	delete speed_sigma; 
	
	std::vector< std::vector< double >* >* speeds;
	std::vector< std::vector< double >* >* sigmas;
	
	for(unsigned int i = 0; i < num_iterations; i++)
	{
		speed_sigma = expectation
			(
				*prev_speeds, 
				*prev_sigmas,
		 		*phi_est,
				*phi_sigma_est
			);
		
		speeds = speed_sigma->first;
		sigmas = speed_sigma->second;
		delete speed_sigma;

//		print_speeds(speeds, sigmas, updates_emkf, net);

		delete_phi(phi_est);
		delete_phi(phi_sigma_est);
		
		phi_sigma = maximization(*speeds, *sigmas);

		phi_est = phi_sigma->first;
		phi_sigma_est = phi_sigma->second;
		delete phi_sigma;
		
//		print_phi(phi_est, phi_sigma_est, net);

		output_file << i << " " << log_likelihood(*speeds, 
			*sigmas, *phi_est, *phi_sigma_est) << "\n";
		
		delete_speeds(prev_speeds);
		delete_speeds(prev_sigmas);

		prev_speeds = speeds;
		prev_sigmas = sigmas;
	}
	
//	print_speeds(prev_speeds, prev_sigmas, updates_emkf, net);

	delete_speeds(prev_speeds);
	delete_speeds(prev_sigmas);
	output_file.close();

	return (new std::pair<t_phi*,t_phi*>(phi_est, phi_sigma_est));
}

void EMKalman::set_missing_speeds
	(
		std::vector<double>& speeds, 
		const double avg_speed_k,
		const double start_speed,
		const double start_sigma,
		const std::vector<double>& prev_speeds, 
		const std::vector<double>& prev_sigmas, 
		const unsigned int num,
		const unsigned int time,
		const std::vector< std::pair< unsigned int, emkf_update_info* > * >& traj,
		const t_phi& phi_est, const t_phi& sigma_phi_est,
		const std::vector<double>& vars_phi,
		const double sigma_trans
	)
{
	unsigned int seg_from;
	unsigned int seg_to;
	
	double sigma;
	
	sigma =  time * sigma_trans + SMALLDOUBLE;

	Program qp (CGAL::EQUAL, true, 0, false, 0); 

	for(unsigned int i = 0; i < num; i++)
	{
		//std::cout << "qp.set_a(" << i << ",0,1);" << std::endl;;
		qp.set_a(i,0,1);
	}

	//std::cout << "qp.set_b(0," << num * avg_speed_k << ");" << std::endl;
	qp.set_b(0, num * avg_speed_k);

	double phi;
	
	double coeff = (double) avg_speed_k / pow(sigma, 2);
	
	if(speeds.size() > 0)
	{
		seg_from = traj.at(speeds.size()-1)->first;
		seg_to = traj.at(speeds.size())->first;
		
		if(phi_est.at(seg_from)->find(seg_to) != 
			phi_est.at(seg_from)->end())
		{
			phi = phi_est.at(seg_from)->at(seg_to);
		
			coeff += (double) phi*speeds.back() / vars_phi[0];
		}
		else
		{
			coeff += (double) prev_speeds[speeds.size()] / vars_phi[0];
		}
	}
	else
	{
		coeff += (double) prev_speeds[speeds.size()] / vars_phi[0];
	}

	//std::cout << "qp.set_c(0," << -coeff << ");" << std::endl;
	qp.set_c(0,-coeff);
	
	coeff = (double) 1 / (2*pow(sigma, 2));
	
	coeff += (double) 1 / (2 * vars_phi[0]);
	
	if(num > 1)
	{
		seg_from = traj.at(speeds.size())->first;
		seg_to = traj.at(speeds.size()+1)->first;
	
		if(phi_est.at(seg_from)->find(seg_to) != 
			phi_est.at(seg_from)->end())
		{
			phi = phi_est.at(seg_from)->at(seg_to);

			coeff += (double) pow(phi, 2) / (2* vars_phi[1]);
		}
	}
	
	//std::cout << "qp.set_d(0,0," << 2*coeff << ");" << std::endl;
	qp.set_d(0,0, 2*coeff);
	
	for(unsigned int i = 1; i < num; i++)
	{
		seg_from = traj.at(speeds.size()+i-1)->first;
		seg_to = traj.at(speeds.size()+i)->first;
		
		if(phi_est.at(seg_from)->find(seg_to) != 
			phi_est.at(seg_from)->end())
		{
			phi = phi_est.at(seg_from)->at(seg_to);
			coeff = (double) phi / vars_phi[i];
			//std::cout << "qp.set_d(" << i << "," << i-1 << "," << -coeff  << ");" << std::endl;
			qp.set_d(i, i-1,-coeff);
		
			coeff = (double) avg_speed_k / pow(sigma, 2);
			//std::cout << "qp.set_c(" << i << "," << -coeff  << ");" << std::endl;
			qp.set_c(i,-coeff);
		}
		else
		{
			coeff = (double) prev_speeds[i] / vars_phi[i]
				+ (double) avg_speed_k / pow(sigma, 2);
			
			//std::cout << "qp.set_c(" << i << "," << -coeff  << ");" << std::endl;
			qp.set_c(i,-coeff);
		}
		
		coeff = (double) 1 / (2 * vars_phi[i])
			+ (double) 1 / (2 * pow(sigma,2));

		if(i < num - 1)
		{
			seg_from = traj.at(speeds.size()+i)->first;
			seg_to = traj.at(speeds.size()+i+1)->first;
			
			if(phi_est.at(seg_from)->find(seg_to) != 
				phi_est.at(seg_from)->end())
			{
				phi = phi_est.at(seg_from)->at(seg_to);

				coeff += (double) pow(phi, 2) / (2* vars_phi[i+1]);
			}
		}

		//std::cout << "qp.set_d(" << i << "," << i << "," << 2*coeff << ");" << std::endl;
		qp.set_d(i, i, 2*coeff);
	}
	
	//CGAL::print_nonnegative_quadratic_program(std::cout, qp, "problem");
	Solution s = CGAL::solve_nonnegative_quadratic_program(qp, ET());
//	std::cout << s << std::endl;

	Solution::Variable_value_iterator it = s.variable_values_begin();

	while( it != s.variable_values_end())
	{
		speeds.push_back(to_double(*it));
		//std::cout << to_double(*it) << std::endl;		
		++it;
	}
}

void EMKalman::kalman_filter_comp
		(
			const std::vector< std::pair< unsigned int, emkf_update_info* > * >& traj,
			const std::vector< double >& prev_speeds,
			const std::vector< double >& prev_sigmas,
			const t_phi& phi_est, const t_phi& sigma_phi_est,
			const double sigma_trans, std::list<emkf_up*>& emkf_comp
		)
{
	std::vector< double >* speeds_comp = new std::vector<double>;
	speeds_comp->reserve(traj.size());
	double speed_phi;
	unsigned int num = 0;
	unsigned int seg_from;
	unsigned int seg_to;
	emkf_update_info* up;
	double time_comp = 0;
			
	speed_phi = prev_speeds[0];
	
	for(unsigned int s = 0; s < traj.size(); s++)
	{
		if(s > 0)
		{
			seg_from = traj.at(s-1)->first;
			seg_to = traj.at(s)->first;
			
			if(phi_est.at(seg_from)->find(seg_to) != 
				phi_est.at(seg_from)->end())
			{
				speed_phi = speed_phi * phi_est.at(seg_from)->at(seg_to);
			}
			else
			{
				speed_phi = prev_speeds[s];
			}
		}
		
		speeds_comp->push_back(speed_phi);
		num++;

		if(traj.at(s)->second != NULL)
		{
			up = traj.at(s)->second;
			
			for(unsigned int i = 0; i < num; i++)
			{
				if(speeds_comp->at(s-i) > 0)
				{
					time_comp += (double) net->segment_length(traj.at(s-i)->first) / speeds_comp->at(s-i);
				}
			}

			if(fabs(up->total_time-time_comp) >= max_error || s == traj.size() - 1)
			{
				add_update(emkf_comp, traj.at(s)->first, up->total_time);
				speed_phi = up->avg_speed;
				time_comp = up->total_time;
			}

			num = 0;
		}
	}
	
	delete speeds_comp;
}

std::pair< std::vector< double >*, std::vector<double>* >*	
	EMKalman::kalman_filter
		(
			const std::vector< std::pair< unsigned int, emkf_update_info* > * >& traj,
			const std::vector< double >& prev_speeds,
			const std::vector< double >& prev_sigmas,
			const t_phi& phi_est, const t_phi& sigma_phi_est,
			const double sigma_trans
		)
{
	std::vector< double >* speeds = new std::vector<double>;
	std::vector<double>* sigmas = new std::vector<double>;
	std::vector<double> vars_phi;
	speeds->reserve(traj.size());
	sigmas->reserve(traj.size());
	vars_phi.reserve(traj.size());
	double avg_phi = 0;
	double avg_phi_var = 0;
	double speed_phi;
	double var_phi;
	double avg_up;
	double var_up;
	unsigned int num = 0;
	double K;
	unsigned int seg_from;
	unsigned int seg_to;
	emkf_update_info* up;
	double avg_speed_k;
	double var_speed_k;
	double sigma;
			
	speed_phi = prev_speeds[0];
	var_phi = pow(prev_sigmas[0], 2);
	
	double start_speed = prev_speeds[0];
	double start_sigma = prev_sigmas[0];
	
	for(unsigned int s = 0; s < traj.size(); s++)
	{
		if(num == 0)
		{
			start_speed = speed_phi;
			start_sigma = sqrt(var_phi);
		}

		if(s > 0)
		{
			seg_from = traj.at(s-1)->first;
			seg_to = traj.at(s)->first;
			
			if(phi_est.at(seg_from)->find(seg_to) != 
				phi_est.at(seg_from)->end())
			{
				var_phi = phi_est.at(seg_from)->at(seg_to)
					* var_phi 
					* phi_est.at(seg_from)->at(seg_to)
					+ pow(sigma_phi_est.at(seg_from)->at(seg_to), 2);
				speed_phi = speed_phi * phi_est.at(seg_from)->at(seg_to);
			}
			else
			{
				var_phi = var_phi + pow(prev_sigmas[s], 2);
				speed_phi = prev_speeds[s];

				start_speed = speed_phi;
				start_sigma = sqrt(var_phi);
			}
		}
		
		vars_phi.push_back(var_phi);

		avg_phi += speed_phi;
		avg_phi_var += var_phi;
		num++;

		if(traj.at(s)->second != NULL)
		{
			up = traj.at(s)->second;
			avg_up = up->avg_speed;

			if(num > 0)
			{
				avg_phi = (double) avg_phi / num;
				avg_phi_var = (double) avg_phi_var / num;
			}
			else
			{
				avg_phi = 0;
				avg_phi_var = std::numeric_limits<double>::max();
			}

			var_up = pow(up->sigma, 2);

			if(avg_phi_var + var_up > 0)
			{
				K = (double) avg_phi_var / (avg_phi_var + var_up);
			}
			else
			{
				K = 0;
			}

			avg_speed_k = avg_phi + K * (avg_up - avg_phi);

			if(num < 10)
			{
				set_missing_speeds(*speeds, avg_speed_k, start_speed, start_sigma, 
					prev_speeds, prev_sigmas, num, up->time, traj, phi_est, 
					sigma_phi_est, vars_phi, sigma_trans);
				
				var_speed_k = var_phi;
				sigma = up->time * sigma_trans + SMALLDOUBLE;
				
				for(unsigned int i = 0; i < num; i++)
				{
					var_phi = vars_phi[i];
	
					if(var_phi + sigma > 0)
					{
						K = (double) var_phi / (var_phi + pow(sigma, 2));
					}
					else
					{
						K = 0;
					}
				
					var_speed_k = var_phi - K * var_phi;
					sigmas->push_back(sqrt(var_speed_k));
				}
			}
			else
			{
				var_speed_k = avg_phi_var - K * avg_phi_var;
				sigma = sqrt(var_speed_k);

				for(unsigned int i = 0; i < num; i++)
				{
					speeds->push_back(
						avg_speed_k
					);

					sigmas->push_back(
						sigma
					);
				}
			}
			
			
			var_phi = var_speed_k;
			vars_phi.clear();
			num = 0;
			avg_phi = 0;
			avg_phi_var = 0;
			speed_phi = speeds->back();
		}
	}
	
	return (new std::pair< std::vector< double >*, std::vector<double>* >(speeds, sigmas));
}


pthread_param_emkf* new_pthread_param_emkf
	(
		std::vector < std::vector< std::pair< unsigned int, emkf_update_info* > * > * >* updates_emkf,
		std::vector< std::vector< double >* >* prev_speeds,
		std::vector< std::vector< double >* >* prev_sigmas,
		std::vector< std::vector< double >* >* speeds,
		std::vector< std::vector< double >* >* sigmas,
		t_phi* phi_est,
		t_phi* phi_sigma_est,
		unsigned int* pointer,
		pthread_mutex_t* mutex_pool,
		const double sigma_trans
	)
{
	pthread_param_emkf* param = new  pthread_param_emkf;
	param->updates_emkf = updates_emkf;
	param->prev_speeds = prev_speeds;
	param->prev_sigmas = prev_sigmas;
	param->speeds = speeds;
	param->sigmas = sigmas;
	param->phi_est = phi_est;
	param->phi_sigma_est = phi_sigma_est;
	param->pointer = pointer;
	param->mutex_pool = mutex_pool;
	param->sigma_trans = sigma_trans;

	return param;
}


void run_thread_kalman_filter
	(
		std::vector < std::vector< std::pair< unsigned int, emkf_update_info* > * > * >* updates_emkf,
		std::vector< std::vector< double >* >* prev_speeds,
		std::vector< std::vector< double >* >* prev_sigmas,
		std::vector< std::vector< double >* >* speeds,
		std::vector< std::vector< double >* >* sigmas,
		t_phi* phi_est,
		t_phi* phi_sigma_est,
		unsigned int* pointer,
		pthread_mutex_t* mutex_pool,
		const double sigma_trans
	)
{
	unsigned int t;
	std::pair< std::vector< double >*, std::vector<double>* >* speed_sigma_pair;
	std::vector< std::pair< unsigned int, emkf_update_info* > * >* traj;
	
	while(true)
	{
		pthread_mutex_lock(mutex_pool);

		if(*pointer == updates_emkf->size())
		{
			 pthread_mutex_unlock(mutex_pool);
			 break;
		}
		else
		{
			t = *pointer;
			*pointer = *pointer + 1;
			pthread_mutex_unlock(mutex_pool);
		}

		traj = updates_emkf->at(t);

		speed_sigma_pair = EMKalman::kalman_filter
			(
				*traj,
				*(prev_speeds->at(t)), 
				*(prev_sigmas->at(t)), 
				*phi_est, 
				*phi_sigma_est,
				sigma_trans
			);

		speeds->at(t) = speed_sigma_pair->first;
		sigmas->at(t) = speed_sigma_pair->second;

		delete speed_sigma_pair;
	}
}

void* start_thread_kalman_filter(void* v_param)
{
	pthread_param_emkf* param = (pthread_param_emkf*) v_param;
	run_thread_kalman_filter
		(
			param->updates_emkf, param->prev_speeds, param->prev_sigmas,
			param->speeds, param->sigmas, param->phi_est, param->phi_sigma_est,
			param->pointer, param->mutex_pool, param->sigma_trans
		);

	pthread_exit(NULL);
}

std::pair< std::vector< std::vector< double >* >*, std::vector< std::vector< double > * >* >*
	EMKalman::expectation
		(
			std::vector< std::vector< double >* >&
			prev_speeds,
			std::vector< std::vector< double >* >&
			prev_sigmas,
			t_phi& phi_est, t_phi& phi_sigma_est
		)
{
	std::vector< std::vector< double >* >* speeds = new std::vector< std::vector< double >* >;
	std::vector< std::vector< double >* >* sigmas = new std::vector< std::vector< double >* >;
	speeds->reserve(prev_speeds.size());
	sigmas->reserve(prev_speeds.size());

	for(unsigned int t = 0; t < updates_emkf.size(); t++)
	{
		speeds->push_back(NULL);
		sigmas->push_back(NULL);
	}
		
	pthread_t* threads = (pthread_t*) malloc (num_threads * sizeof(pthread_t));
	pthread_mutex_t* mutex_pool = new pthread_mutex_t;
	pthread_mutex_init(mutex_pool, NULL);
	unsigned int pointer = 0;
	pthread_param_emkf* param;
	std::vector<pthread_param_emkf*> params;
	
	for(unsigned int t = 0; t < num_threads; t++)
	{
		param = new_pthread_param_emkf
			(
				&updates_emkf,
				&prev_speeds,
				&prev_sigmas,
				speeds,
				sigmas,
				&phi_est,
				&phi_sigma_est,
				&pointer,
				mutex_pool,
				sigma_trans
			);

		params.push_back(param);
		pthread_create(&threads[t], NULL, start_thread_kalman_filter, param);
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
	
	return (new std::pair< std::vector< std::vector< double >* >*, std::vector< std::vector< double > * >* >
		(speeds, sigmas));
}

std::pair<t_phi*, t_phi*>* EMKalman::maximization
	(
		const std::vector< std::vector< double >* >& speeds,
		const std::vector< std::vector< double >* >& sigmas
	)
		const
{
	t_phi* sums = new t_phi;
	t_phi* counts = new t_phi;
	t_phi* phi_est = new t_phi;
	t_phi* phi_sigma_est = new t_phi;

	unsigned int s2;
	
	for(unsigned int s1 = 0; s1 < net->size(); s1++)
	{
		const std::list<unsigned int>* neigh = net->neighbors(s1);
		
		phi_est->insert
			(
				std::pair<unsigned int, 
					std::map < unsigned int, double > * > 
				(s1, new std::map < unsigned int, double > )
			);

		phi_sigma_est->insert
			(
				std::pair<unsigned int, 
					std::map < unsigned int, double > * > 
				(s1, new std::map < unsigned int, double > )
			);

		sums->insert
			(
				std::pair<unsigned int, 
					std::map < unsigned int, double > * > 
				(s1, new std::map < unsigned int, double > )
			);
		
		counts->insert
			(
				std::pair<unsigned int, 
					std::map < unsigned int, double > * > 
				(s1, new std::map < unsigned int, double > )
			);

		for(std::list<unsigned int>::const_iterator it = neigh->begin();
			it != neigh->end(); ++it)
		{
			s2 = *it;

			phi_est->at(s1)->insert(std::pair<unsigned int, double > (s2, 0));
			phi_sigma_est->at(s1)->insert(std::pair<unsigned int, double > (s2, 0));
			sums->at(s1)->insert(std::pair<unsigned int, double > (s2, 0));
			counts->at(s1)->insert(std::pair<unsigned int, double > (s2, 0));
		}
	}

	unsigned int seg_from;
	unsigned int seg_to;
	std::vector< std::pair< unsigned int, emkf_update_info* > * >* traj;
	
	for(unsigned int t = 0; t < updates_emkf.size(); t++)
	{
		traj = updates_emkf.at(t);
		
		for(unsigned int s = 1; s < traj->size(); s++)
		{
			seg_from = traj->at(s-1)->first;
			seg_to = traj->at(s)->first;
			
			if(phi_est->at(seg_from)->find(seg_to) != 
				phi_est->at(seg_from)->end())
			{
				phi_est->at(seg_from)->at(seg_to) += speeds.at(t)->at(s);
				sums->at(seg_from)->at(seg_to) += speeds.at(t)->at(s-1);
				counts->at(seg_from)->at(seg_to) += 1;
			}
		}
	}
	
	for(unsigned int s1 = 0; s1 < net->size(); s1++)
	{
		const std::list<unsigned int>* neigh = net->neighbors(s1);
		
		for(std::list<unsigned int>::const_iterator it = neigh->begin();
			it != neigh->end(); ++it)
		{
			s2 = *it;
			
			if(sums->at(s1)->at(s2) > 0)
			{
				phi_est->at(s1)->at(s2) = (double) phi_est->at(s1)->at(s2) 
					/  sums->at(s1)->at(s2);
			}
			else
			{
				phi_est->at(s1)->at(s2) = 1;
			}
		}
	}
	
	double diff;
	for(unsigned int t = 0; t < updates_emkf.size(); t++)
	{
		traj = updates_emkf.at(t);
		
		for(unsigned int s = 1; s < traj->size(); s++)
		{
			seg_from = traj->at(s-1)->first;
			seg_to = traj->at(s)->first;
			
			if(phi_est->at(seg_from)->find(seg_to) != 
				phi_est->at(seg_from)->end())
			{
				diff = phi_est->at(seg_from)->at(seg_to) * speeds.at(t)->at(s-1)
					- speeds.at(t)->at(s); 
				
				phi_sigma_est->at(seg_from)->at(seg_to) += pow(diff, 2.0);
			}
		}
	}
	
	for(unsigned int s1 = 0; s1 < net->size(); s1++)
	{
		const std::list<unsigned int>* neigh = net->neighbors(s1);
		
		for(std::list<unsigned int>::const_iterator it = neigh->begin();
			it != neigh->end(); ++it)
		{
			s2 = *it;
			
			if(counts->at(s1)->at(s2) > 0)
			{
				phi_sigma_est->at(s1)->at(s2) = sqrt((double) phi_sigma_est->at(s1)->at(s2) 
					/  counts->at(s1)->at(s2)) + SMALLDOUBLE;
			}
			else
			{
				phi_sigma_est->at(s1)->at(s2) = 0; 
			}
		}
	}

	delete_phi(counts);
	delete_phi(sums);

	return (new std::pair<t_phi*, t_phi*>(phi_est, phi_sigma_est));
}

EMKalman::~EMKalman()
{
	delete_phi(phi);
	delete_phi(phi_sigma);
}

double EMKalman::log_likelihood
	(
		const std::vector< std::vector< double >* >& speeds,
		const std::vector< std::vector< double >* >& speed_sigmas,
		const t_phi& phi_est, const t_phi& phi_sigma_est
	) const
{
	double log_like = 0;
	std::vector< std::pair< unsigned int, emkf_update_info* > * >* traj;
	unsigned int seg_from;
	unsigned int seg_to;
	double tmp;
	double var;

	for(unsigned int t = 0; t < updates_emkf.size(); t++)
	{
		traj = updates_emkf.at(t);

		for(unsigned int s = 1; s < traj->size(); s++)
		{
			seg_from = traj->at(s-1)->first;
			seg_to = traj->at(s)->first;

			var = speed_sigmas[t]->at(0);
			
			if(phi_est.at(seg_from)->find(seg_to) != 
				phi_est.at(seg_from)->end())
			{
				var = phi_est.at(seg_from)->at(seg_to) * var * phi_est.at(seg_from)->at(seg_to)
					+  pow(phi_sigma_est.at(seg_from)->at(seg_to), 2);
				
				if(var > 0)
				{
					tmp = -(double) pow(phi_est.at(seg_from)->at(seg_to) * speeds[t]->at(s-1) 
						- speeds[t]->at(s), 2) 
						/ (2.0 * var);
					log_like += tmp;
					
					tmp = -log(sqrt(var) * sqrt(2 * PI));
					log_like += tmp;
				}
			}
		}
	}
	
	return log_like;
}


