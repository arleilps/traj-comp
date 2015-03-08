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
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/IterativeLinearSolvers>

/*my includes*/
#include "traj_comp.h"
#include "io.h"

const double LeastSquares::w = 0.5;
const unsigned int LeastSquares::d0 = 2;

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
	node->next = 0;		//FIXME should be at least adjacent to segment
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

		compress(dist_times, comp_dist_times);

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

double equal_double(const double d1, const double d2)
{
	return (fabs(d1 - d2) <= 1.0e-12);
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

	if((p_i.dist < (double) tan_to * p_i.time + b_to
		&& p_i.dist > (double) tan_from * p_i.time + b_from)
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

	if((p_i.dist < (double) tan_to * p_i.time + b_to
		&& p_i.dist > (double) tan_from * p_i.time + b_from)
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

void LeastSquares::train(const std::string training_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	Trajectory::read_trajectories(trajectories, training_traj_file_name, net);
	train_t->start();
	
	y.reserve(1000 * trajectories.size());
	Q.reserve(1000 * trajectories.size());
	
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		_num_traj_train++;
		_num_updates_train += traj->size();
		
		traj->get_sparse_rep(Q, y, sz, net);
		delete traj;
	}
	
	laplacian_affinity_matrix();
	least_squares_regression();

	train_t->stop();
	_training_time = train_t->get_seconds();
}

void Hybrid::train(const std::string training_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	Trajectory::read_trajectories(trajectories, training_traj_file_name, net);
	train_t->start();
	
	y.reserve(1000 * trajectories.size());
	Q.reserve(1000 * trajectories.size());
	
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		_num_traj_train++;
		_num_updates_train += traj->size();
		
		traj->get_sparse_rep(Q, y, sz, net);
	}
	
	laplacian_affinity_matrix();
	least_squares_regression();
	compute_lsq_sigmas(trajectories);

	train_t->stop();
	_training_time = train_t->get_seconds();
}
	
void Hybrid::compute_lsq_sigmas(const std::list<Trajectory*>& trajectories)
{
	Trajectory* traj;
	std::vector<unsigned int> counts;
	lsq_sigmas.reserve(net->size());
	counts.reserve(net->size());

	for(unsigned int s = 0; s < f.size(); s++)
	{
		lsq_sigmas.push_back(0);
		counts.push_back(0);
	}

	for(std::list<Trajectory*>::const_iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		traj->get_lsq_sigmas
			(
				lsq_sigmas,
				counts,
				f,
				net
			);
	}

	for(unsigned int s = 0; s < f.size(); s++)
	{
		lsq_sigmas[s] = sqrt((double) lsq_sigmas[s] / (counts[s] - 1));
	}

}

void LeastSquares::test(const std::string test_traj_file_name)
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
		traj->get_dist_times_least_squares(dist_times, f, net);
		
		compress(comp_dist_times, dist_times, traj);

		Trajectory::delete_dist_times(dist_times);
		Trajectory::delete_dist_times(comp_dist_times);
		delete traj;
	}
	
	_compression_time = comp_t->get_seconds();
}

void Hybrid::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	std::list < dist_time* > comp_dist_times;
	
	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);

	//Compresses each trajectory in the file and computes the total
	//number of updates
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;

		compress(comp_dist_times, traj);

		Trajectory::delete_dist_times(comp_dist_times);
		delete traj;
	}
	
	_compression_time = comp_t->get_seconds();
}

void LeastSquares::compress
	(
		std::list < dist_time* >& comp_dist_times, 
		const std::list < dist_time* >& dist_times,
		Trajectory* traj
	) 
{
	comp_t->start();
	std::list < dist_time* > pred_dist_times;
	dist_time* dt;
	dist_time* dt_pred;
		
	//traj->print();
	
	traj->get_pred_dist_times_least_squares
		(
			pred_dist_times, 
			f, net
		);

	std::list<dist_time*>::const_iterator it = dist_times.begin();
	std::list<dist_time*>::iterator it_pred = pred_dist_times.begin();
	dt = *it;
	++it;
	++it_pred;
	comp_dist_times.push_back(new_dist_time(dt->dist, dt->time));
	double fix = 0;
	double time_pred;

	while(it != dist_times.end())
	{
		dt = *it;
		dt_pred = *it_pred;
		time_pred = dt_pred->time + fix; 
		
		++it;
		++it_pred;
		
		if(fabs(dt->time - time_pred) > max_error 
			|| it == dist_times.end())
		{
			comp_dist_times.push_back(new_dist_time(dt->dist, dt->time));
			fix = dt->time - time_pred;
		}

	}
	
	/*
	for(std::list<dist_time*>::iterator jt = pred_dist_times.begin(); 
		jt != pred_dist_times.end(); ++jt)
	{
		std::cout << (*jt)->dist << " , " << (*jt)->time << std::endl;
	}
		
	std::cout << std::endl;
	
	for(std::list<dist_time*>::const_iterator jt = dist_times.begin(); 
		jt != dist_times.end(); ++jt)
	{
		std::cout << (*jt)->dist << " , " << (*jt)->time << std::endl;
	}
	
	std::cout << std::endl;
		
	for(std::list<dist_time*>::iterator jt = comp_dist_times.begin(); 
		jt != comp_dist_times.end(); ++jt)
	{
		std::cout << (*jt)->dist << " , " << (*jt)->time << std::endl;
	}
	*/
	Trajectory::delete_dist_times(pred_dist_times);
	
	comp_t->stop();
	_num_updates_orig += dist_times.size();
	_num_updates_comp += comp_dist_times.size();
	_num_traj_comp++;
}

void Hybrid::compress
	(
		std::list < dist_time* >& comp_dist_times, 
		Trajectory* traj
	)
{
}

void LeastSquares::least_squares_regression()
{
	Eigen::SparseMatrix<double> QS(net->size(), sz);
	QS.setFromTriplets(Q.begin(), Q.end());
	Q.clear();
	Eigen::SparseMatrix<double> QST(QS.transpose());
	Eigen::SparseMatrix<double> LS(net->size(), net->size());
	LS.setFromTriplets(L.begin(), L.end());
	L.clear();
	Eigen::SparseMatrix<double> A(QS * QST + lambda * LS);
	Eigen::Map<Eigen::VectorXd> y_vec(y.data(), y.size());
	Eigen::VectorXd b(QS * y_vec);
	Eigen::ConjugateGradient<Eigen::SparseMatrix<double> > cg;
	cg.compute(A);
	f.resize(net->size());
	f = cg.solve(b);

//	std::cout << f << std::endl;
	
	for(unsigned int i = 0; i < f.size(); i++)
	{
		if(f[i] != 0)
		{
			f[i] = (double) net->segment_length(i) / f[i];
		}
	}

	std::cout << f << std::endl;
}

void LeastSquares::laplacian_affinity_matrix()
{
	std::vector < std::map<unsigned int, unsigned int>* > distances;
	std::vector < std::map<unsigned int, double>* > weights;
	distances.reserve(net->size());
	weights.reserve(net->size());
	double weight;
	std::map<unsigned int, unsigned int>::iterator rev;
	L.reserve(net->size() * 100);

	for(unsigned int s = 0; s < net->size(); s++)
	{
		distances.push_back(new std::map<unsigned int, unsigned int>);
		weights.push_back(new std::map<unsigned int, double>);
		net->get_num_hops_from(*(distances.at(s)), s, d0);
	}

	for(unsigned int s = 0; s < net->size(); s++)
	{
		for(std::map<unsigned int, unsigned int>::iterator it = 
			distances.at(s)->begin(); it != distances.at(s)->end();
			++it)
		{
			rev = distances.at(it->first)->find(s);

			if(rev != distances.at(it->first)->end())
			{
				if(rev->second < it->second)
				{
					weight = pow(w, rev->second);
				}
				else
				{
					weight = pow(w, it->second);
				}
			}
			else
			{
				weight = pow(w, it->second);
			}
			
			if(s < it->first)
			{
				weights.at(s)->insert(std::pair<unsigned int, double>(it->first, weight));
				weights.at(it->first)->insert(std::pair<unsigned int, double>(s, weight));
			}
		}
	}

	double sum_weights;
	for(unsigned int s = 0; s < net->size(); s++)
	{
		sum_weights = 0;
		for(std::map<unsigned int, double>::iterator it = 
			weights.at(s)->begin(); it != weights.at(s)->end();
			++it)
		{
			if(s != it->first)
			{
				L.push_back(Eigen::Triplet<double>(s, it->first, -1*(it->second)));
				sum_weights += it->second;
			}
		}

		L.push_back(Eigen::Triplet<double>(s, s, sum_weights));
	}
	
	for(unsigned int s = 0; s < net->size(); s++)
	{
		delete distances.at(s);
		delete weights.at(s);
	}
}


