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

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

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

void CompTrajectory::add_update
	(
		const unsigned int segment,
                const unsigned int time,
                const double dist,
                const unsigned int id
        )
{
	Trajectory::add_update(segment, time, dist);
	back()->id = id;
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

void OntracFull::train(const std::string& training_traj_file_name)
{
	ppm->train(training_traj_file_name);
	em->train(training_traj_file_name);
}

const bool OntracFull::insert(const std::string& input_file_name)
{
	std::list<Trajectory*> trajectories;
	std::string obj;
	Trajectory* traj;

	if(Trajectory::read_trajectories(trajectories, input_file_name, net))
	{
		insert_t->start();
		 
		_num_traj_inserted++;
		 
		for(std::list<Trajectory*>::iterator it = trajectories.begin();
		 	it != trajectories.end(); ++it)
		{
		 	traj = *it;
		 	_num_updates_orig += traj->size();
			insert(traj->object(), *traj);
		}
		 
		insert_t->stop();
		  
		_insert_time = insert_t->get_seconds();
		  
		Trajectory::delete_trajectories(&trajectories);
		
		return true;
	}
	else
	{
		return false;
	}
}

CompTrajectory* OntracFull::compress(Trajectory* traj)
{
	CompTrajectory* comp_traj = new CompTrajectory();
	
	std::vector < std::vector< std::pair< unsigned int, em_update_info* > * > * > updates_em;
	traj->get_em_rep(updates_em, sigma_gps, net);
	std::vector< std::pair< unsigned int, em_update_info* > * > * traj_em = updates_em.at(0);

	Trajectory::iterator it = traj->begin();
	comp_traj->add_update((*it)->segment, (*it)->time, (*it)->dist, 0);
	unsigned int next;
	em_update_info* up;
	double time = em->avg_times->at(traj->front()->segment);
	double total_time = traj->front()->time + time;
	double fused_time = traj->front()->time + time;
	double error = pow(em->sigma_times->at(traj->front()->segment), 2);
	double K;
	unsigned int s = 1;
	std::list<unsigned int> segs;
	std::list<unsigned int> ids;
	std::list<unsigned int>::iterator itd;
	double t;
	double d = 0;
	bool added = false;

	next = ppm->next_segment(it, traj, ppm->tree);
	++it;

	while(it != traj->end())
	{
		up = traj_em->at(s)->second;
		time += em->avg_times->at(traj_em->at(s)->first);
		error += pow(em->sigma_times->at(traj_em->at(s)->first), 2);
		d = d + net->segment_length(traj_em->at(s)->first);
		
		if(it != traj->end() && (*it)->segment != next)
		{	
			segs.push_back((*it)->segment);
			ids.push_back(s);

			if(up != NULL)
			{
				added = true;
			}
		}
		
		if(up != NULL)
		{
			K = (double) error / (error + pow(up->sigma, 2));
			fused_time = fused_time + time + K * (up->time - time);
			
			if(fabs(total_time+time-fused_time) > max_error || s == traj_em->size()-1)
			{
				itd = ids.begin();
				for(std::list<unsigned int>::iterator its = segs.begin();
					its != segs.end(); ++its)
				{
					t = (double) total_time + ((time + K * (up->time - time)) * net->segment_length(*its))
						/ d;
					comp_traj->add_update(*its, t, 0, *itd);
					++itd;
				}

				if(! added)
				{
					comp_traj->add_update((*it)->segment, fused_time, 0, s);
				}

				total_time = fused_time;
			}
			else
			{
				itd = ids.begin();
				for(std::list<unsigned int>::iterator its = segs.begin();
					its != segs.end(); ++its)
				{
					t = total_time + (double) (time * net->segment_length(*its)) / d;
					comp_traj->add_update(*its, t, 0, *itd);
					++itd;
				}
				
				total_time += time;
			}

			segs.clear();
			ids.clear();
			d = 0;
			
			time = 0;
			error = 0;
			added = false;
		}

		s++;  
		next = ppm->next_segment(it, traj, ppm->tree);
		++it;
	}

	return comp_traj;
}

const bool OntracFull::insert(const std::string& obj, Trajectory& traj)
{
	CompTrajectory* comp_traj = compress(&traj);

	bool status;

	for(Trajectory::iterator it = comp_traj->begin();
		it != comp_traj->end(); ++it)
	{
		if(! TrajDB::insert(obj, *(*it)))
		{
			status = false;
		}
	}

	return status;
}


seg_time* OntracPart::where_at
	(
		const std::string& obj,
		const unsigned int time
	) const
{
	Trajectory* traj = decompress_partial
		(obj, time);

	em->decompress(traj);

	Trajectory::iterator it = traj->end();
	--it;
	
	while(it != traj->begin() && (*it)->time > time)
	{
		--it;
	}

	return (*it);
}

seg_time* OntracFull::where_at
	(
		const std::string& obj,
		const unsigned int time
	) const
{
	Trajectory* traj = ppm->decompress
		(db->get_traj(obj));

	em->decompress(traj);

	Trajectory::iterator it = traj->end();
	--it;

	while(it != traj->begin() && (*it)->time > time)
	{
		--it;
	}

	return (*it);
}

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
	if(r <= order)
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
				(next->second)++;
				
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

			if(itj != traj->begin())
			{
				add_trajectory(iti, --itj, traj, node, r+1);
			}
		}
		else
		{
			node = new_node_ppm((*itj)->segment);
			tree->children.insert(std::pair<unsigned int, NodePPM*>((*itj)->segment, node));
			node->freq.insert(std::pair<unsigned int, double>((*iti)->segment, 1));
			node->freq_next = 1;
			node->next = (*iti)->segment;
		
			if(itj != traj->begin())
			{
				add_trajectory(iti, --itj, traj, node, r+1);
			}
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

	if(node_it != tree->children.end())
	{
		node = node_it->second;

		if(it != traj->begin())
		{
			return next_segment(--it, traj, node);
		}
		else
		{
			return node->next;
		}
	}
	else
	{
		return tree->next;
	}
}

void PredPartMatch::next_segment_set
	(
		Trajectory::iterator it, 
		Trajectory* traj,
		NodePPM* tree,
		std::map<unsigned int, bool>& set,
		bool all
	) const
{
	std::map<unsigned int, NodePPM*>::iterator node_it;
	NodePPM* node;
	
	if(all)
	{
		for(node_it = tree->children.begin(); node_it != tree->children.end(); ++node_it)
		{
			set.insert(std::pair<unsigned, bool>(node_it->second->next, true));
			node = node_it->second;
			next_segment_set(it, traj, node, set, true);
		}
	}
	else
	{
		node_it = tree->children.find((*it)->segment);
	
		if(node_it != tree->children.end())
		{
			set.insert(std::pair<unsigned, bool>(node_it->second->next, true));
		
			if(it != traj->begin())
			{
				node = node_it->second;
				next_segment_set(--it, traj, node, set, false);
			}
		}
	}
}

const unsigned int PredPartMatch::count_next_paths
	(
		Trajectory::iterator it,
		Trajectory* traj,
		NodePPM* tree,
		const unsigned int target,
		const unsigned int n,
		const unsigned int num_hops,
		std::map<unsigned int, bool>& new_target
	) const
{
	if(n > num_hops)
	{
		return 0;
	}

	if(n == num_hops && traj->back()->segment == target)
	{
		return 1;
	}
		
	unsigned int num;
	unsigned int total = 0;
	std::map<unsigned int, bool> nexts;
	Trajectory* new_traj;
	Trajectory::iterator new_it;
	next_segment_set(it, traj, tree, nexts, false);

	for(std::map<unsigned int, bool>::iterator itj = nexts.begin();
		itj != nexts.end(); ++itj)
	{
		new_traj = new Trajectory(*traj);
		new_traj->add_update(itj->first, 0, 0);
		new_it = new_traj->end();
		--new_it;

		num = count_next_paths(new_it, new_traj, tree, target, n+1, num_hops, new_target);
		
		if(n == order)
		{
			new_target.insert(std::pair<unsigned int, bool>((*it)->segment, true));
		}

		total += num;
		delete new_traj;
	}

	return total;
}

Trajectory* OntracPart::decompress_partial
	(
		const std::string& obj,
		const unsigned int time
	) const
{
	Trajectory* traj = new Trajectory();
	const seg_time* st = db->where_at(obj, time);
	const seg_time* prev = db->where_at(obj, st->time-1);
	std::map<unsigned int, bool>* target = new std::map<unsigned int, bool>;
	std::map<unsigned int, bool>* new_target = new std::map<unsigned int, bool>;
	
	unsigned int num_hops = st->id - prev->id;
	traj->add_update(prev->segment, prev->time, 0);
	Trajectory::iterator it = traj->begin();
	target->insert(std::pair<unsigned int, bool>(st->segment,true));
	unsigned int num;

	while(true)
	{
		num = 0;
		for(std::map<unsigned int,bool>::iterator jt = target->begin(); 
			jt != target->end(); ++jt)
		{
			num += ppm->count_next_paths
				(it, traj, ppm->tree, jt->first, 0,
				num_hops, *new_target);
		}

		if(num > 1)
		{
			st = prev;
			prev = db->where_at(obj, (prev->time)-1);
				
			if(prev == NULL)
			{
				traj = db->get_traj(obj);
				return ppm->decompress(traj);
			}
			else
			{
				num_hops = st->id - prev->id + 1;
				target->clear();
				delete target;
				target = new_target;
				new_target = new std::map<unsigned int, bool>;
				delete traj;
				traj->add_update(prev->segment, prev->time, 0);
				it = traj->begin();
			}
		}
		else
		{
			traj = db->get_traj(obj, prev->time);
			break;
		}
	}

	return traj;
}

CompTrajectory* PredPartMatch::compress(Trajectory* traj)
{
	comp_t->start();
	
	CompTrajectory* comp_traj = new CompTrajectory();
	
	Trajectory::iterator it = traj->begin();
	comp_traj->add_update((*it)->segment, (*it)->time, (*it)->dist);
	unsigned int next;
	unsigned int i = 0;

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

		i++;
	}
	
	comp_t->stop();
	_num_updates_orig += traj->size();
	_num_updates_comp += comp_traj->size();
	_num_traj_comp++;

	return comp_traj;
}

Trajectory* PredPartMatch::decompress
	(
		Trajectory* comp_traj
	)
{
	Trajectory* traj = new Trajectory;
	unsigned int next;

	Trajectory::iterator iti = comp_traj->begin();
	Trajectory::iterator itj;

	traj->add_update
		(
			(*iti)->segment, 
			(*iti)->time, (*iti)->dist
		);
	
	traj->back()->id = (*iti)->id;
	
	++iti;
	while(iti != comp_traj->end())
	{
		itj = traj->end();
		--itj;
		
		if((*iti)->id - (*itj)->id == 1)
		{
			traj->add_update
			(
				(*iti)->segment, 
				(*iti)->time, (*iti)->dist
			);
			
			traj->back()->id = (*iti)->id;

			++iti;
		}
		else
		{
			next = next_segment(itj, traj, tree);
			traj->add_update(next, 0, 0);
			traj->back()->id = (*itj)->id + 1;
		}
	}

	return traj;
}

void PredPartMatch::extend
	(
		Trajectory* traj,
		const std::vector<double>& avg_times
	)
{
	Trajectory::iterator itj;
	unsigned int next;

	itj = traj->end();
	--itj;

	next = next_segment(itj, traj, tree);
	double time = traj->back()->time + avg_times.at(next);
	traj->add_update(next, time, 0);
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
	shortest_path_comp = new ShortestPath(net, shortest_path_file_name);

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


void delete_em_info
	(
		std::vector < std::vector< std::pair< unsigned int, em_update_info* > * > * >& updates_em
	)
{
	std::vector< std::pair< unsigned int, em_update_info* > * >* traj;

	for(unsigned int t = 0; t < updates_em.size(); t++)
	{
		traj = updates_em.at(t);

		for(unsigned int i = 0; i < traj->size(); i++)
		{
			delete traj->at(i)->second;
			delete traj->at(i);
		}

		delete traj;
	}

	updates_em.clear();
}

void print_model(std::vector<double>* avg_times, std::vector<double>* sigma_times, RoadNet* net)
{
	std::cout << "PRINT MODEL" << std::endl;
	for(unsigned int s = 0; s < net->size(); s++)
	{
		if(avg_times->at(s) > 0)
		{
			std::cout << "avg(" << net->seg_name(s) << ") = " << avg_times->at(s) << std::endl;
			std::cout << "sigma(" << net->seg_name(s) << ") = " << sigma_times->at(s) << std::endl;
		}
	}
}

void EM::avg_sigma_speed()
{
	std::vector< std::pair< unsigned int, em_update_info* > * >* traj;
	avg_speed = 0;
	sigma_speed = 0;
	sigma_trans = 0;
	unsigned int num = 0;
	double prev_ratio;
	double ratio;
	double speed;

	for(unsigned int t = 0; t < updates_em.size(); t++)
	{
		traj = updates_em.at(t);
		prev_ratio = 0;

		for(unsigned int s = 1; s < traj->size(); s++)
		{
			if(traj->at(s)->second != NULL)
			{
				
				if(traj->at(s)->second->time > 0)
				{
					speed = (double) traj->at(s)->second->dist / traj->at(s)->second->time;
					avg_speed += speed;
				}
				else
				{
					speed = 0;
				}

				if(speed > 0)
				{
					ratio = (double)  1 / speed;
				}
				else
				{
					ratio = 0;
				}
				
				sigma_trans += pow(prev_ratio-ratio, 2);

				prev_ratio = ratio;

				num++;
			}
		}
	}

	avg_speed = (double) avg_speed / num;
	sigma_trans = sqrt((double) sigma_trans / num);

	for(unsigned int t = 0; t < updates_em.size(); t++)
	{
		traj = updates_em.at(t);

		for(unsigned int s = 0; s < traj->size(); s++)
		{
			if(traj->at(s)->second != NULL)
			{
				if(traj->at(s)->second->time > 0)
				{
					speed = (double) traj->at(s)->second->dist / traj->at(s)->second->time;
				}
				else
				{
					speed = 0;
				}

				
				sigma_speed += pow(avg_speed 
					- speed, 2);
			}
		}

		sigma_speed = sqrt((double) sigma_speed / num);
	}
}

void print_update_info(
	std::vector < std::vector< std::pair< unsigned int, em_update_info* > * > * >&
	updates_em
)
{
	std::vector< std::pair< unsigned int, em_update_info* > * >* traj;
	em_update_info* up;

	for(unsigned int t = 0; t < updates_em.size(); t++)
	{
		traj = updates_em.at(t);

		for(unsigned int s = 0; s < traj->size(); s++)
		{
			up = traj->at(s)->second;

			if(up != NULL)
			{
				std::cout << std::endl;
				std::cout << "dist: " << up->dist << std::endl;
				std::cout << "total_dist: " << up->total_dist << std::endl;
				std::cout << "time: " << up->time << std::endl;
				std::cout << "total_time: " << up->total_time << std::endl;
				std::cout << "sigma: " << up->sigma << std::endl;
			}
		}
	}		
}

void EM::train(const std::string training_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	Trajectory::read_trajectories(trajectories, training_traj_file_name, net);
	Trajectory::break_trajectories(trajectories, net);

	updates_em.reserve(trajectories.size());
	train_t->start();
	
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		_num_updates_train += traj->size();
		traj->get_em_rep(updates_em, sigma_gps, net);
		delete traj;
	}

	_num_traj_train = updates_em.size();
	
	avg_sigma_speed();
	std::pair<std::vector<double>* , std::vector<double>* >* avg_sigma_pair = em();
	avg_times = avg_sigma_pair->first;
	
	
	sigma_times = avg_sigma_pair->second;
	
	delete_em_info(updates_em);

	train_t->stop();
	_training_time = train_t->get_seconds();
}

void EM::test(const std::string test_traj_file_name)
{
	std::list<Trajectory*> trajectories;
	Trajectory* traj;
	CompTrajectory* comp;
	
	Trajectory::read_trajectories(trajectories, test_traj_file_name, net);
	updates_em.reserve(trajectories.size());

	//Compresses each trajectory in the file and computes the total
	//number of updates
	comp_t->start();
	for(std::list<Trajectory*>::iterator it = trajectories.begin();
		it != trajectories.end(); ++it)
	{
		traj = *it;
		traj->get_em_rep(updates_em, sigma_gps, net);

		comp = compress(*(updates_em.back()), *traj);
		
		delete comp;
		delete traj;
	}
	
	comp_t->stop();
	
	delete_em_info(updates_em);
	
	_compression_time = comp_t->get_seconds();
}

CompTrajectory* EM::compress
	(
		std::vector< std::pair< unsigned int, em_update_info* > * >& traj,
		Trajectory& trajj
	) 
{
	em_update_info* up;
	unsigned int seg;
	double time = avg_times->at(trajj.front()->segment);
	double total_time = trajj.front()->time + time;
	double fused_time = trajj.front()->time + time;
	double error = pow(sigma_times->at(trajj.front()->segment), 2);
	double K;
	CompTrajectory* comp = new CompTrajectory();

	comp->add_update(trajj.front()->segment, trajj.front()->time, 0);

	for(unsigned int s = 1; s < traj.size(); s++)
	{
		seg = traj.at(s)->first;
		up = traj.at(s)->second;
		time += avg_times->at(seg);
		error += pow(sigma_times->at(seg), 2);
		
		if(up != NULL)
		{
			K = (double) error / (error + pow(up->sigma, 2));
			fused_time = fused_time + time + K * (up->time - time);
			total_time += time;

			if(fabs(total_time-fused_time) > max_error)
			{
				comp->add_update(seg, fused_time, 0);
				total_time = fused_time;
			}
			
			time = 0;
			error = 0;
		}
	}

	_num_updates_orig += traj.size();
	_num_updates_comp += comp->size();
	_num_traj_comp++;

	return comp;
}

void EM::decompress(Trajectory* traj) const
{
	Trajectory::iterator itj;
	Trajectory::iterator iti = traj->begin(); 
	double time;
	double pred_time = 0;
	double total_time;
	
	while(iti != traj->end())
	{
		itj = iti;
		pred_time = 0;
		
		++itj;

		if(itj != traj->end())
		{
			while((*itj)->dist == 0 && (*itj)->time == 0)
			{
				pred_time += avg_times->at((*itj)->segment);
				++itj;
			}
				
			pred_time += avg_times->at((*itj)->segment);

			time = (*itj)->time - (*iti)->time;
			total_time = (*iti)->time;
	
			++iti;

			while((*iti)->dist == 0 && (*iti)->time == 0)
			{
				total_time += (double) (time * avg_times->at((*iti)->segment)) / pred_time; 
				(*iti)->time = total_time;
				++iti;
			}
		}
		else
		{
			break;
		}
	}
}

std::vector< std::vector< double >* >* EM::start_times() const
{
	std::vector< std::vector< double >* >*
		times = new std::vector< std::vector< double >* >;
	
	std::vector< std::pair< unsigned int, em_update_info* > * >* traj;
	em_update_info* up;
	unsigned int num;
	double time;
	unsigned int seg;

	times->reserve(_num_traj_train);

	for(unsigned int t = 0; t < updates_em.size(); t++)
	{
		traj = updates_em.at(t);
		times->push_back(new std::vector<double>);
		num = 1;

		for(unsigned int s = 1; s < traj->size(); s++)
		{
			num++;
			
			if(traj->at(s)->second != NULL)
			{
				up = traj->at(s)->second;
				
				for(unsigned int i = 0; i < num; i++)
				{
					seg = traj->at(s-i)->first;
					time = (double) (up->time * net->segment_length(seg)) / up->dist;
					times->at(t)->push_back(time);
				}
				
				num = 0;
			}
		}
	}

	return times;
}

void delete_times(std::vector< std::vector< double >* >* times)
{
	for(unsigned int i = 0; i < times->size(); i++)
	{
		delete times->at(i);
	}

	delete times;
}

void print_times(std::vector< std::vector< double >* >* times)
{
	for(unsigned int t = 0; t < times->size(); t++)
	{
		for(unsigned int s = 0; s < times->at(t)->size(); s++)
		{
			std::cout << times->at(t)->at(s) << " ";
		}

		std::cout << std::endl;
	}

}

std::pair< std::vector<double> *, std::vector<double> * >* EM::start_model() const
{
	std::vector< std::vector<double>* >* times = start_times();
	std::pair< std::vector<double> *, std::vector<double> * >* avg_sigma;
		
	avg_sigma = maximization(*times);

	delete_times(times);

	return avg_sigma;
}

std::pair< std::vector<double> *, std::vector<double>* >* EM::em()
{
	std::pair< std::vector<double> *, std::vector<double> * >* avg_sigma = start_model();
	std::vector<double> * _avg_times = avg_sigma->first;
	std::vector<double> * _sigma_times = avg_sigma->second;
	
	delete avg_sigma;
	std::ofstream output_file(output_file_name.c_str(), std::ios::out);

	std::vector< std::vector< double >* >* times;
	
	for(unsigned int i = 0; i < num_iterations; i++)
	{
		
		times = expectation
			(
		 		*_avg_times,
				*_sigma_times
			);
		
		delete _avg_times;
		delete _sigma_times;
		
		avg_sigma = maximization(*times);

		_avg_times = avg_sigma->first;
		_sigma_times = avg_sigma->second;
		delete avg_sigma;
	
		output_file << i << " " << log_likelihood(*times, 
			*_avg_times, *_sigma_times) << std::endl;
		
		delete_times(times);
	}

	output_file.close();
	
	return (new std::pair<std::vector<double>* , std::vector<double>* >(_avg_times, _sigma_times));
}

pthread_param_em* new_pthread_param_em
	(
		std::vector<double>* avg_times,
		std::vector<double>* sigma_times,
		std::vector< std::vector< double >* >* times,
		unsigned int* pointer,
		std::vector < std::vector< std::pair< unsigned int, em_update_info* > * > * >* updates_em,
		pthread_mutex_t* mutex_pool,
		double sigma_trans,
		RoadNet* net
	)
{
	pthread_param_em* param = new pthread_param_em;
	
	param->avg_times = avg_times;
	param->sigma_times = sigma_times;
	param->times = times;
	param->pointer = pointer;
	param->updates_em = updates_em;
	param->mutex_pool = mutex_pool;
	param->sigma_trans = sigma_trans;
	param->net = net;

	return param;
}

std::vector<double>*
	EM::gaussian_model(
		const std::vector< std::pair< unsigned int, em_update_info* > * >& traj,
		const std::vector<double>& _avg_times,
		const std::vector<double>& _sigma_times,
		double sigma_trans,
		const RoadNet* net
	)
{
	unsigned int seg;
	unsigned int prev_seg;
	em_update_info* up;
	std::vector<double>* times = new std::vector<double>;
	times->reserve(traj.size());
	
	double** D = (double**) malloc (traj.size() * sizeof(double*));
	double* C = (double*) malloc (traj.size() * sizeof(double));

	for(unsigned int s = 0; s < traj.size(); s++)
	{
		D[s] = (double*) malloc (traj.size() * sizeof(double));
		C[s] = 0;

		for(unsigned int t = 0; t < traj.size(); t++)
		{
			D[s][t] = 0;
		}
	}
	
	for(unsigned int s = 0; s < traj.size(); s++)
	{
		seg = traj.at(s)->first;
		D[s][s] += (double) 1 / (2 * pow(_sigma_times[seg], 2));
		C[s] -= (double) _avg_times[seg] / pow(_sigma_times[seg], 2);

		if(s > 0)
		{
			D[s-1][s] -= (double) 1 / ((double) pow(sigma_trans, 2) * net->segment_length(prev_seg) * net->segment_length(seg));
		}

		if(s != 0 && s != (traj.size() - 1))
		{
			D[s][s] += (double) 1 / pow((double) sigma_trans * net->segment_length(seg), 2);
		}
		else
		{
			D[s][s] += (double) 1 / (2 * pow((double) sigma_trans * net->segment_length(seg), 2));
		}

		prev_seg = seg;
	}
	
	unsigned int num = 1;

	for(unsigned int s = 1; s < traj.size(); s++)
	{
		up = traj.at(s)->second;
		num++;

		if(up != NULL)
		{
			for(unsigned int i = 0; i < num; i++)
			{
				D[s-i][s-i] += (double) 1 / (2 * pow(up->sigma, 2));
				C[s-i] -= (double) up->time /  pow(up->sigma, 2);

				for(unsigned int j = 0; j < num; j++)
				{
					if(s-i < s-j)
					{
						D[s-i][s-j] += (double) 1 /  pow(up->sigma, 2);
					}
				}
			}

			num = 0;
		}
	}

	IloEnv   env;
	IloModel model(env);

	IloNumVarArray x(env);
	IloExpr obj(env);
	
	for(unsigned int s = 0; s < traj.size(); s++)
	{
		x.add(IloNumVar(env, 0.0, IloInfinity));
	}
	
	for(unsigned int s = 0; s < traj.size(); s++)
	{
		obj +=  C[s] * x[s];
	}

	for(unsigned int s = 0; s < traj.size(); s++)
	{
		obj +=  D[s][s] * x[s]*x[s];
		
		for(unsigned int t = 0; t < s; t++)
		{
			obj += D[t][s] * x[t]*x[s];
		}
	}
	
	free(C);

	for(unsigned int s = 0; s < traj.size(); s++)
	{
		free(D[s]);
	}

	free(D);
	IloCplex cplex(model);
	
	cplex.setParam(IloCplex::Threads, 1);

	cplex.setOut(env.getNullStream());
	model.add(IloMinimize(env, obj));
	
	try
	{
		cplex.solve();

		for(unsigned int s = 0; s < traj.size(); s++)
		{
			times->push_back(cplex.getValue(x[s]));
		}
	}
	catch (IloException& e)
	{
		unsigned int num = 1;
		double t;

		for(unsigned int s = 1; s < traj.size(); s++)
		{
			num++;
			
			if(traj.at(s)->second != NULL)
			{
				up = traj.at(s)->second;
				
				for(unsigned int i = 0; i < num; i++)
				{
					seg = traj.at(s-i)->first;
					t = (double) (up->time * net->segment_length(seg)) / up->dist;
					times->push_back(t);
				}
				
				num = 0;
			}
		}
	}
	
	x.end();
	obj.end();
	cplex.end();
	env.end();

	return times;
}


void run_thread_em	
	(
		std::vector<double>* avg_times,
		std::vector<double>* sigma_times,
		std::vector< std::vector< double >* >* times,
		unsigned int* pointer,
		std::vector < std::vector< std::pair< unsigned int, em_update_info* > * > * >* updates_em,
		pthread_mutex_t* mutex_pool,
		double sigma_trans,
		RoadNet* net
	)
{
	unsigned int s;
	std::vector< std::pair< unsigned int, em_update_info* > * >* traj;
	std::vector< double >* t;
	
	while(true)
	{
		pthread_mutex_lock(mutex_pool);
	 
	 	if(*pointer == updates_em->size())
	 	{
	 		pthread_mutex_unlock(mutex_pool);
	 		break;
	 	}
	 	else
	 	{
			s = *pointer;
			traj = updates_em->at(s);
	 		*pointer = *pointer + 1;
	 		pthread_mutex_unlock(mutex_pool);
	 	}
		
		t = EM::gaussian_model(*traj, *avg_times, *sigma_times, sigma_trans, net);
		
		pthread_mutex_lock(mutex_pool);
	 	times->at(s) = t;
		pthread_mutex_unlock(mutex_pool);
	}
}

void* start_thread_em(void* v_param)
{
	pthread_param_em* param = (pthread_param_em*) v_param;
	run_thread_em(param->avg_times, param->sigma_times, param->times,
		param->pointer,	param->updates_em, param->mutex_pool, param->sigma_trans, param->net);

	pthread_exit(NULL);
}

std::vector< std::vector< double >* >*
	EM::expectation
		(
			std::vector<double>& _avg_times, std::vector<double>& _sigma_times
		)
{
	std::vector< std::vector< double >* >* times = new std::vector< std::vector< double >* >;
	times->reserve(updates_em.size());
	
	for(unsigned int t = 0; t < updates_em.size(); t++)
	{
		times->push_back(NULL);
	}

	pthread_t* threads = (pthread_t*) malloc (num_threads * sizeof(pthread_t));
	pthread_param_em* param;
	pthread_mutex_t* mutex_pool = new pthread_mutex_t;
	pthread_mutex_init(mutex_pool, NULL);
	unsigned int pointer = 0;
	std::vector<pthread_param_em*> params;

	for(unsigned int t = 0; t < num_threads; t++)
	{
		param = new_pthread_param_em(&_avg_times, &_sigma_times, times, 
			&pointer, &updates_em, mutex_pool, sigma_trans, net);
	 	params.push_back(param);
	 	pthread_create(&threads[t], NULL, start_thread_em, param);
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
		
	return times;
}

std::pair< std::vector<double>*, std::vector<double> * >*
	EM::maximization
	(
		const std::vector< std::vector< double >* >& times
	)
		const
{
	std::vector<double>* _avg_times = new std::vector<double>;
	std::vector<double>* _sigma_times = new std::vector<double>;
	std::vector<double>* count = new std::vector<double>;
	unsigned int seg;

	for(unsigned int s = 0; s < net->size(); s++)
	{
		_avg_times->push_back(0);
		_sigma_times->push_back(0);
		count->push_back(0);
	}

	std::vector< std::pair< unsigned int, em_update_info* > * >* traj;
	
	for(unsigned int t = 0; t < updates_em.size(); t++)
	{
		traj = updates_em.at(t);
		
		for(unsigned int s = 0; s < traj->size(); s++)
		{
			seg = traj->at(s)->first;
			_avg_times->at(seg) += times.at(t)->at(s);
			count->at(seg) += 1;
		}
	}
	
	for(unsigned int s = 0; s < net->size(); s++)
	{
		if(count->at(s) > 0)
		{
			_avg_times->at(s) = (double) _avg_times->at(s) / count->at(s);
		}
		else
		{
			_avg_times->at(s) = (double) net->segment_length(s) / avg_speed;
		}
	}

	for(unsigned int t = 0; t < updates_em.size(); t++)
	{
		traj = updates_em.at(t);
		
		for(unsigned int s = 0; s < traj->size(); s++)
		{
			seg = traj->at(s)->first;
			_sigma_times->at(seg) += pow(_avg_times->at(seg)-times.at(t)->at(s), 2);
		}
	}

	for(unsigned int s = 0; s < net->size(); s++)
	{
		if(count->at(s) > 0)
		{
			_sigma_times->at(s) = 1 + sqrt((double) _sigma_times->at(s) / count->at(s));
		}
		else
		{
			_sigma_times->at(s) = (double) net->segment_length(s) / Trajectory::min_speed();
		}
	}

	delete count;

	return (new std::pair< std::vector<double> *, std::vector<double> * >(_avg_times, _sigma_times));
}

EM::~EM()
{
	delete avg_times;
	delete sigma_times;
}

double EM::log_likelihood
	(
		const std::vector< std::vector< double >* >& times,
		const std::vector<double>& _avg_times, 
		const std::vector<double>& _sigma_times
	) const
{
	double s_log_like = 0;
	double log_like;
	std::vector< std::pair< unsigned int, em_update_info* > * >* traj;
	unsigned int seg;
	unsigned int prev_seg = 0;
	double up_time;
	em_update_info* up;
	double r1;
	double r2;

	for(unsigned int t = 0; t < updates_em.size(); t++)
	{
		traj = updates_em.at(t);
		up_time = 0;
		log_like = 0;

		for(unsigned int s = 0; s < traj->size(); s++)
		{
			seg = traj->at(s)->first;
			
			log_like += (double) pow(times.at(t)->at(s)-_avg_times.at(seg), 2) 
				/  (2.0 * pow(_sigma_times.at(seg), 2)) + log(_sigma_times.at(seg));
			
			up_time += times.at(t)->at(s);
			
			if(s > 0)
			{
				
				r1 = (double) times.at(t)->at(s) /  net->segment_length(seg);
				r2 = (double) times.at(t)->at(s-1) /  net->segment_length(prev_seg);

				log_like += (double) pow(r1-r2, 2) / (2.0 * pow(sigma_trans, 2));
			
				up = traj->at(s)->second;
				
				if(up != NULL)
				{	
					log_like += (double) pow(up_time - up->time, 2) 
						/ (2.0 * pow(up->sigma, 2));

					up_time = 0;
				}
			}

			prev_seg = seg;
		}
		
//		std::cout << "ll = " << log_like << std::endl;
//		std::cout << "gm = " << gaussian_model_value(*traj, _avg_times, _sigma_times, *(times.at(t))) << std::endl;
		s_log_like += log_like;
	}
	
	return s_log_like;
}

double EM::gaussian_model_value(
		const std::vector< std::pair< unsigned int, em_update_info* > * >& traj,
		const std::vector<double>& _avg_times,
		const std::vector<double>& _sigma_times,
		const std::vector< double >& times
	) const
{
	unsigned int seg;
	unsigned int prev_seg;
	em_update_info* up;
	
	double** D = (double**) malloc (traj.size() * sizeof(double*));
	double* C = (double*) malloc (traj.size() * sizeof(double));

	for(unsigned int s = 0; s < traj.size(); s++)
	{
		D[s] = (double*) malloc (traj.size() * sizeof(double));
		C[s] = 0;

		for(unsigned int t = 0; t < traj.size(); t++)
		{
			D[s][t] = 0;
		}
	}
	
	
	for(unsigned int s = 0; s < traj.size(); s++)
	{
		seg = traj.at(s)->first;
		D[s][s] += (double) 1 / (2 * pow(_sigma_times[seg], 2));
		C[s] -= (double) _avg_times[seg] / pow(_sigma_times[seg], 2);

		if(s > 0)
		{
			D[s-1][s] -= (double) 1 / ((double) pow(sigma_trans, 2) * net->segment_length(prev_seg) * net->segment_length(seg));
		}

		if(s != 0 && s != (traj.size() - 1))
		{
			D[s][s] += (double) 1 / pow((double) sigma_trans * net->segment_length(seg), 2);
		}
		else
		{
			D[s][s] += (double) 1 / (2 * pow((double) sigma_trans * net->segment_length(seg), 2));
		}

		prev_seg = seg;
	}
	
	unsigned int num = 1;

	double obj = 0;
	
	for(unsigned int s = 1; s < traj.size(); s++)
	{
		up = traj.at(s)->second;
		num++;

		if(up != NULL)
		{
			for(unsigned int i = 0; i < num; i++)
			{
				D[s-i][s-i] += (double) 1 / (2 * pow(up->sigma, 2));
				C[s-i] -= (double) up->time /  pow(up->sigma, 2);

				for(unsigned int j = 0; j < num; j++)
				{
					if(s-i < s-j)
					{
						D[s-i][s-j] += (double) 1 /  pow(up->sigma, 2);
					}
				}
			}
			
			obj += (double) pow(up->time, 2) / (2 * pow(up->sigma, 2));
			num = 0;
		}
	}


	for(unsigned int s = 0; s < traj.size(); s++)
	{
		obj +=  C[s] * times.at(s);
	}

	for(unsigned int s = 0; s < traj.size(); s++)
	{
		seg = traj.at(s)->first;
		
		obj +=  D[s][s] * times.at(s) * times.at(s);
		
		for(unsigned int t = 0; t < s; t++)
		{
			obj += D[t][s] * times.at(t) * times.at(s);
		}

		obj += (double) pow(_avg_times.at(seg), 2) / (2 * pow(_sigma_times.at(seg), 2));
	}
	
	free(C);

	for(unsigned int s = 0; s < traj.size(); s++)
	{
		free(D[s]);
	}

	free(D);

	return obj;
}
