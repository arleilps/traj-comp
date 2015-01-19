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

NodeSubt* FreqSubt::new_node()
{
	NodeSubt* node = new NodeSubt;
	node->id = id++;
	node->freq = 0;
	node->seg = 0;
	node->depth = 0;
	node->children = new std::map<unsigned int, NodeSubt*>;
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

//Frequent subtrajectories are identified in the training file by building
//a frequent subtrajectory tree, which is similar to a frequent substring tree
const unsigned int FreqSubt::train(const std::string training_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory::read_trajectories(trajectories, training_traj_file_name, net);
	
	//Adds each trajectory into the tree
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		add_trajectory(*it);

		delete *it;
	}

	//Prunes unfrequent subtrajectories based on the given minimum support
	prune_unfrequent_subtraj();

	//Sets an index for the frequent subtrajectories that contain a given segment
	set_seg_index();

	return size_tree;
}

const unsigned int FreqSubt::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	unsigned int updates = 0;
	Trajectory* traj;
	CompTrajectory* comp_traj;
	std::list<NodeSubt*> compressed;

	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);

	//Compresses each trajectory in the file and computes the total
	//number of updates
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

//Each node in the tree has a map from a segment id to a child node
void FreqSubt::add_trajectory
	(
		Trajectory::iterator it, 
		Trajectory* traj, 
		NodeSubt* tree, 
		const unsigned int depth,
		std::list<NodeSubt*>* suffix_pointers,
		std::list<NodeSubt*>* new_suffix_pointers
	)
{
	tree->freq++;
	
	if(it != traj->end() && depth < max_length)
	{
		//Searches for the segment in the map
		std::map<unsigned int, NodeSubt*>::iterator node = tree->children->find((*it)->segment);
		
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
			tree->children->insert(std::pair<unsigned int, NodeSubt*>((*it)->segment, new_node()));
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
	std::list<NodeSubt*>* suffix_pointers = new std::list<NodeSubt*>;
	std::list<NodeSubt*>* new_suffix_pointers;
	
	suffix_pointers->push_back(NULL);

	//Trajectory added by iterating over it
	Trajectory::iterator iti = traj->end();
	--iti;
	
	//Doesn't look good.
	while(true)
	{
		itj = iti;
		new_suffix_pointers = new std::list<NodeSubt*>;
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
	NodeSubt* curr_node = tree;
	std::map<unsigned int, NodeSubt*>::iterator node_it;
	std::list<NodeSubt*> decomp;
	std::list<unsigned int> start_times;
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
			start_times.push_back((*it)->start_time);
			curr_node = node_it->second;
			++it;
			size_dec++;
		}
	}

	unsigned int l = 0;
	unsigned int start_time;

	while(size_dec > 0)
	{
		curr_node = decomp.front();
		start_time = start_times.front();
		decomp.pop_front();
		start_times.pop_front();
		size_dec--;

		if(l == 0)
		{
			comp_traj->add_update(curr_node->id, start_time, start_time);
			l =  curr_node->depth - 1;
		}
		else
		{
			l--;
		}
	}

	comp_traj->set_end_times();

	comp_traj->back()->end_time = traj->back()->end_time;

	return comp_traj;
}

void FreqSubt::delete_tree(NodeSubt* node)
{
	for(std::map<unsigned int, NodeSubt*>::iterator it = node->children->begin(); 
		it != node->children->end(); ++it)
	{
		delete_tree(it->second);
	}

	delete node->children;
	delete node;
	size_tree--;
}

void FreqSubt::prune_tree(NodeSubt* root)
{
	NodeSubt* node;
	
	std::map<unsigned int, NodeSubt*>* new_map = new std::map<unsigned int, NodeSubt*>; 

	std::map<unsigned int, NodeSubt*>::iterator it = root->children->begin(); 
	 
	while(it != root->children->end())
	{
		node = it->second;

		if(node->freq < min_sup)
		{
			delete_tree(node);
		}
		else
		{
			new_map->insert(std::pair<unsigned int, NodeSubt*>(it->first, node));
			prune_tree(node);
		}
		
		++it;
	}

	delete root->children;
	root->children = new_map;
}

void FreqSubt::prune_unfrequent_subtraj()
{
	NodeSubt* node;

	for(std::map<unsigned int, NodeSubt*>::iterator it = tree->children->begin();
		it != tree->children->end(); ++it)
	{
		node = it->second;
		prune_tree(node);
	}
}

void FreqSubt::set_seg_index(NodeSubt* root)
{
	NodeSubt* node;
	
	for(std::map<unsigned int, NodeSubt*>::iterator it = root->children->begin(); 
		it != root->children->end(); ++it)
	{
		node = it->second;
		seg_to_freq_subt_index.at(node->seg)->push_back(node->id);
		set_seg_index(node);
	}
}

void FreqSubt::set_seg_index()
{
	NodeSubt* node;

	for(std::map<unsigned int, NodeSubt*>::iterator it = tree->children->begin();
		it != tree->children->end(); ++it)
	{
		node = it->second;
		set_seg_index(node);
	}
}

void FreqSubt::print_tree(NodeSubt* node)
{
	std::string str;
	
	for(std::map<unsigned int, NodeSubt*>::iterator it = node->children->begin(); 
		it != node->children->end(); ++it)
	{
		str = "[" + to_string(net->seg_name(it->first)) + "]";
		print_tree(it->second, str);
	}
}

void FreqSubt::print_tree(NodeSubt* node, const std::string str) 
{
	std::string new_str;
	
	std::cout << str << " " << node->freq << std::endl;

	for(std::map<unsigned int, NodeSubt*>::iterator it = node->children->begin(); 
		it != node->children->end(); ++it)
	{
		new_str = str +"-["+ to_string(net->seg_name(it->first)) + "]";
		print_tree(it->second, new_str);
	}
}

void FreqSubt::freq_sub_traj
	(
		std::list<std::pair<unsigned int, Trajectory * > * >& fsts,
		NodeSubt* node, 
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

		for(std::map<unsigned int, NodeSubt*>::iterator it = node->children->begin(); 
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
	for(std::map<unsigned int, NodeSubt*>::iterator it = tree->children->begin(); 
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

NodePPM* PredPartMatch::new_node_ppm(const unsigned int segment)
{
	NodePPM* node = new NodePPM;
	
	node->segment = segment;
	node->id = size_tree++;
	node->next = 0;		//FIXME should be at least adjacent to segment
	node->freq_next = 0;

	return node;
}

const unsigned int PredPartMatch::train(const std::string training_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory::read_trajectories(trajectories, training_traj_file_name, net);
	
	//Adds each trajectory into the tree
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		add_trajectory(*it);

		delete *it;
	}

	return size_tree;
}

const unsigned int PredPartMatch::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	unsigned int updates = 0;
	Trajectory* traj;
	CompTrajectory* comp_traj;
	std::list<NodeSubt*> compressed;
	
	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);

	//Compresses each trajectory in the file and computes the total
	//number of updates
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

CompTrajectory* PredPartMatch::compress(Trajectory* traj) const
{
	CompTrajectory* comp_traj = new CompTrajectory();
	
	Trajectory::iterator it = traj->begin();
	comp_traj->add_update((*it)->segment, (*it)->start_time, (*it)->end_time);
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
					(*it)->start_time, 
					(*it)->end_time
				);
		}
	}

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

const unsigned int ShortestPath::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	unsigned int updates = 0;
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
		updates += comp_traj->size();
		
		delete comp_traj;
	}
	
	Trajectory::delete_trajectories(&trajectories);

	return updates;
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

CompTrajectory* ShortestPath::compress(Trajectory* traj) const
{
	CompTrajectory* comp_traj = new CompTrajectory();
		
	Trajectory::iterator it = traj->begin();
	unsigned int start = (*it)->segment;
		
	comp_traj->add_update(start, (*it)->start_time, (*it)->end_time);
		
	++it;
	unsigned int b_end = (*it)->segment;
	unsigned int end;
	unsigned int b_start_time = (*it)->start_time;
	unsigned int b_end_time = (*it)->end_time;
	++it;

	while(it != traj->end())
	{
		end = (*it)->segment;
			
		if(! check_sp_through(start, end, b_end))	
		{
			comp_traj->add_update(b_end, b_start_time, b_end_time);
			start = b_end;
		}

		b_end = (*it)->segment;
		b_start_time = (*it)->start_time;
		b_end_time = (*it)->end_time;
		++it;
	}

	comp_traj->add_update(b_end, b_start_time, b_end_time);

	return comp_traj;
}

void ShortestPath::compute_shortest_paths()
{
	short_paths.reserve(net->size());
	
	for(unsigned int s = 0; s < net->size(); s++)
	{
		short_paths.push_back(new std::map<unsigned int, unsigned int>);
	}

	for(unsigned int s = 0; s < net->size(); s++)
	{
		net->fill_short_path_struct(s, max_length, short_paths.at(s));

		std::cout << "size = " << short_paths.at(s)->size() << std::endl;
	}
}

void ShortestPath::delete_shortest_paths()
{
	for(unsigned int s = 0; s < net->size(); s++)
	{
		delete short_paths.at(s);
	}
}

