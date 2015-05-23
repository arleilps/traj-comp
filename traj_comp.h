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
#include "perf.h"

#define PI 3.14159265
#define SMALLDOUBLE 1e-5
#define MAXSIZEOPT 10 

/*Node in the frequent subtrajectory tree*/
typedef struct t_node_subt
{
	unsigned int id;
	unsigned int seg;
	unsigned int freq;
	unsigned int depth;
	std::map<unsigned int, t_node_subt*>* children;
	t_node_subt* suffix;
}node_subt;

typedef std::map< unsigned int, std::map < unsigned int, double > * > t_phi;


/**
 * Implements functionalities for a compressed trajectory manipulation
 * so far, nothing beyond standard trajectories (see moving_obj.h)
**/
class CompTrajectory: public Trajectory
{
	public:
		CompTrajectory():Trajectory(){}

		CompTrajectory(const CompTrajectory& traj):Trajectory(traj){};

		virtual ~CompTrajectory(){};
};

typedef struct t_angle
{
	double from;
	double to;
}angle;

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
		**/
		virtual void train(const std::string training_traj_file_name){};
		
		/** 
		 * Tests the compression algorithm using map-matched trajectories in an 
		 * input file.
		 * @param test_traj_file_name test file name
		**/
		virtual void test(const std::string test_traj_file_name){};
		
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
		virtual CompTrajectory* compress(Trajectory* traj) 
		{
			return NULL;
		}

		inline const double compression_time() const
		{
			return _compression_time;
		}

		inline const double training_time() const
		{
			return _training_time;
		}

		inline const double compression_ratio() const
		{
			return (double) _num_updates_orig / _num_updates_comp;
		}

		inline const unsigned int num_updates_orig()  const
		{
			return _num_updates_orig;
		}

		inline const unsigned int num_updates_comp() const
		{
			return _num_updates_comp;
		}

		inline const unsigned int num_traj_comp() const
		{
			return _num_traj_comp;
		}

		inline const unsigned int num_traj_train() const
		{
			return _num_traj_train;
		}

		inline const unsigned int num_updates_train() const
		{
			return _num_updates_train;
		}
	protected:
		RoadNet* net;
		double _compression_time;
		double _training_time;
		unsigned int _num_updates_orig;
		unsigned int _num_updates_comp;
		unsigned int _num_updates_train;
		unsigned int _num_traj_comp;
		unsigned int _num_traj_train;
		ExecTime* comp_t;
		ExecTime* train_t;
};

/**
 * Implements trajectory compression algorithm using frequent subtrajectories.
 * Works in batch-mode.
**/
class FreqSubt: public TrajCompAlgo
{
	friend class ShortestPathFreqSubt;
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
				RoadNet* net,
				const unsigned int _delay=std::numeric_limits<unsigned int>::max()
			)
			:TrajCompAlgo(net)
		{
			delay = _delay;
			min_sup = _min_sup;
			max_length = _max_length;
			tree = new_node();
			node_subt* node;
			size_tree = 0;
			id = 0;
			seg_to_freq_subt_index.reserve(net->size());

			for(unsigned int s = 0; s < net->size(); s++)
			{
				seg_to_freq_subt_index.push_back(new std::list<unsigned int>);
				size_tree++;
				node = new_node();
				node->seg = s;
				node->freq = 0;
				node->depth = 1;
				tree->children->insert(std::pair<unsigned int, node_subt*>(s, node));
			}
		}
		
		/*Destructor*/
		virtual ~FreqSubt()
		{
			delete_tree(tree);

			for(unsigned int s = 0; s < net->size(); s++)
			{
				delete seg_to_freq_subt_index.at(s);
			}
		}

		void train(const std::string training_traj_file_name);

		void test(const std::string test_traj_file_name);

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
		
		CompTrajectory* compress(Trajectory* traj);

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
		node_subt* tree;
		unsigned int size_tree;
		unsigned int id;
		std::vector< std::list<unsigned int>* > seg_to_freq_subt_index;
		unsigned int delay;
		
		/* OBJECT METHODS */
		void print_tree(node_subt* node);
		void print_tree(node_subt* node, const std::string str);
		void delete_tree(node_subt* node);
		
		void freq_sub_traj
			(
				std::list<std::pair<unsigned int, Trajectory * > * >& fsts,
				node_subt* node, 
				Trajectory* traj=NULL
			);

		void prune_unfrequent_subtraj();
		void prune_tree(node_subt* root);
		node_subt* new_node();
		void set_seg_index();
		void set_seg_index(node_subt* root);
		
		void add_trajectory
			(
				Trajectory::iterator it, 
				Trajectory* traj, 
				node_subt* tree,
				const unsigned int depth,
				std::list<node_subt*>* suffix_pointers,
				std::list<node_subt*>* new_suffix_pointers
			);
};

class ShortestPath: public TrajCompAlgo
{
	public:
		ShortestPath
			(
				RoadNet* net,
				const std::string _shortest_path_file_name,
				const unsigned int _delay=std::numeric_limits<unsigned int>::max()
			)
			:TrajCompAlgo(net)
		{
			delay = _delay;
			net->read_short_path_struct(_shortest_path_file_name, short_paths);
		}

		virtual ~ShortestPath()
		{
			delete_shortest_paths();
		}
		
		void train(const std::string training_traj_file_name){};

		void test(const std::string test_traj_file_name);

		void add_trajectory(Trajectory* traj);
		
		CompTrajectory* compress(Trajectory* traj);
	
	private:
		std::vector < std::map < unsigned int , unsigned int > * > short_paths;
		unsigned int delay;
		
		void delete_shortest_paths();
		bool check_sp_through
			(
				const unsigned int start,
				const unsigned int end,
				const unsigned int through
			) const;
};

class ShortestPathFreqSubt: public TrajCompAlgo
{
	public:
		ShortestPathFreqSubt
			(
				const unsigned int min_sup,
				const unsigned int max_length_subt,
				RoadNet* net,
				const std::string _shortest_path_file_name,
				const unsigned int _delay=std::numeric_limits<unsigned int>::max()
			)
				:TrajCompAlgo(net)
		{
			delay = _delay;
			shortest_path_file_name = _shortest_path_file_name;
			freq_subt_comp = new FreqSubt(min_sup, max_length_subt, net);
		}
		
		void train(const std::string training_traj_file_name);

		virtual ~ShortestPathFreqSubt()
		{
			delete freq_subt_comp;
			delete shortest_path_comp;
		}
		
		void test(const std::string test_traj_file_name);

		void add_trajectory(Trajectory* traj){};
		
		CompTrajectory* compress(Trajectory* traj);
	private:
		FreqSubt* freq_subt_comp;
		ShortestPath* shortest_path_comp;
		std::string shortest_path_file_name;
		unsigned int delay;
};

typedef struct t_node_ppm
{
	unsigned int id;
	unsigned int segment;
	unsigned int next;
	double freq_next;
	std::map<unsigned int, t_node_ppm*> children;
	std::map<unsigned int, double> freq;
} NodePPM;

class PredPartMatch: public TrajCompAlgo
{
	public:
		PredPartMatch
			(
				const unsigned int _order, 
				RoadNet* net
			)
				:TrajCompAlgo(net)
			{
				order = _order;
				size_tree = 0;
				tree = new_node_ppm(0);
				NodePPM* node;
			
				for(unsigned int s = 0; s < net->size(); s++)
				{
					node = new_node_ppm(s);
					tree->children.insert(std::pair<unsigned int, NodePPM*>(s, node));
				}
			}

		virtual ~PredPartMatch()
		{
			delete_tree(tree);
		}
		
		void train(const std::string training_traj_file_name);

		void test(const std::string test_traj_file_name);

		void add_trajectory(Trajectory* traj);

		CompTrajectory* compress(Trajectory* traj);
	private:
		unsigned int order;
		unsigned int size_tree;
		NodePPM* tree;

		NodePPM* new_node_ppm
			(
				unsigned int segment
			);
		
		void add_trajectory
			(
				const Trajectory::iterator& iti,
				Trajectory::iterator& itj,
				Trajectory* traj,
				NodePPM* tree,
				const unsigned int r
			);

		void delete_tree(NodePPM* tree);

		const unsigned int next_segment
			(
				Trajectory::iterator it,
				Trajectory* traj,
				NodePPM* tree
			) const;
};

class TSND: public TrajCompAlgo
{
	public:
		TSND(RoadNet* net, const double _max_error)
			:TrajCompAlgo(net)
		{
			max_error = _max_error;
		}

		virtual ~TSND(){};
		
		void test(const std::string test_traj_file_name);
		
		void compress
			(
				std::list < dist_time* >& dist_times,
				std::list < dist_time* >& comp_dist_times
			);
		
		static bool fall_inside
			(
				const angle& R,
				const dist_time& p_index,
				const dist_time& p_i
			);

		static void constrain
			(
				angle& R,
				const dist_time& p_index,
				const dist_time& p_i,
				const double error
			);
	private:
		double max_error;
};

class NSTD: public TrajCompAlgo
{
	public:
		NSTD(const double _max_error, RoadNet* net)
			:TrajCompAlgo(net)
		{
			max_error = _max_error;
		}

		virtual ~NSTD(){};
		
		void test(const std::string test_traj_file_name);
		
		void compress
			(
				std::list < dist_time* >& dist_times,
				std::list < dist_time* >& comp_dist_times,
				Trajectory& traj
			);
		
		static bool fall_inside
			(
				const angle& R,
				const dist_time& p_index,
				const dist_time& p_i
			);

		static void constrain
			(
				angle& R,
				const dist_time& p_index,
				const dist_time& p_i,
				const double error
			);
	private:
		double max_error;
};

typedef struct t_emkf_up
{
	unsigned int seg;
	unsigned int time;
}emkf_up;

class EMKalman: public TrajCompAlgo
{
	public:
		EMKalman(
			const double _max_error,
			RoadNet* net,
			const unsigned int _num_iterations,
			const double _sigma_gps,
			const unsigned int _num_threads,
			const std::string _output_file_name
		)
			:TrajCompAlgo(net)
		{
			max_error = _max_error;
			num_iterations = _num_iterations;
			sigma_gps = _sigma_gps;
			num_threads = _num_threads;
			output_file_name = _output_file_name;
		}

		virtual ~EMKalman();
		
		void train(const std::string training_traj_file_name);

		void test(const std::string test_traj_file_name);
		
		std::pair< std::vector< double >*, std::vector<double>* >*
			kalman_filter(
				const std::vector< std::pair< unsigned int, emkf_update_info* > * >& traj,
				const std::vector< double >& prev_speeds,
				const std::vector< double >& prev_sigmas,
				const t_phi& phi_est, const t_phi& sigma_phi_est,
				const double sigma_trans
			);
	private:
		double max_error;
		unsigned int num_iterations;
		double avg_speed;
		double sigma_speed;
		double sigma_trans;
		double sigma_gps;
		t_phi* phi;
		t_phi* phi_sigma;
		std::vector < std::vector< std::pair< unsigned int, emkf_update_info* > * > * > updates_emkf;
		unsigned int num_threads;
		std::string output_file_name;
		
		std::pair<t_phi*, t_phi*>*
			EM();
		
		std::pair< std::vector< std::vector< double >* >*, std::vector< std::vector< double > * >* >*	
			expectation(
				std::vector< std::vector< double >* >& prev_speeds,
				std::vector< std::vector< double >* >& prev_sigmas,
				t_phi& phi_est, t_phi& phi_sigma_est
			);
		
		
		std::pair<t_phi*, t_phi*>* maximization
			(
				const std::vector< std::vector< double >* >& speeds_est,
				const std::vector< std::vector< double >* >& speeds_sigma
			) const;
		
		void compress
			(
				std::vector< std::pair< unsigned int, emkf_update_info* > * >& traj,
				std::list<emkf_up*>& emkf_comp,
				Trajectory& trajj
			);
		
		std::pair<t_phi*, t_phi*>* start_phi() const;

		std::pair< std::vector< std::vector< double >* >*, std::vector< std::vector< double > * >* >*
			start_speed () const;

		double log_likelihood
			(
				const std::vector< std::vector< double >* >& speeds,
				const std::vector< std::vector< double >* >& speed_sigmas,
				const t_phi& phi_est, const t_phi& phi_sigma_est
			) const;

		void avg_sigma_speed();

		void run_cgal_opt
			(
				std::vector<double>& speeds, 
				const double avg_speed_k,
				const double start_speed,
				const double start_sigma,
				const std::vector<double>& prev_speed,
				const std::vector<double>& prev_sigmas,
				const unsigned int index,
				const unsigned int num,
				const unsigned int time,
				const std::vector< std::pair< unsigned int, emkf_update_info* > * >& traj,
				const t_phi& phi_est, const t_phi& sigma_phi_est,
				const std::vector<double>& vars_phi,
				const double sigma_trans
			);
		
		void set_missing_speeds
			(
				std::vector<double>& speeds, 
				const double avg_speed_k,
				const double start_speed,
				const double start_sigma,
				const std::vector<double>& prev_speed,
				const std::vector<double>& prev_sigmas,
				const unsigned int num,
				const unsigned int time,
				const std::vector< std::pair< unsigned int, emkf_update_info* > * >& traj,
				const t_phi& phi_est, const t_phi& sigma_phi_est,
				const std::vector<double>& vars_phi,
				const double sigma_trans
			);

		void kalman_filter_comp
				(
					const std::vector< std::pair< unsigned int, emkf_update_info* > * >& traj,
					const std::vector< double >& prev_speeds,
					const std::vector< double >& prev_sigmas,
					const t_phi& phi_est, const t_phi& sigma_phi_est,
					const double sigma_trans, std::list<emkf_up*>& emkf_comp
				);
};

typedef struct t_pthread_param_emkf
{
	std::vector < std::vector< std::pair< unsigned int, emkf_update_info* > * > * >* updates_emkf;
	std::vector< std::vector< double >* >* prev_speeds;
	std::vector< std::vector< double >* >* prev_sigmas;
	std::vector< std::vector< double >* >* speeds;
	std::vector< std::vector< double >* >* sigmas;
	t_phi* phi_est;
	t_phi* phi_sigma_est;
	unsigned int* pointer;
	pthread_mutex_t* mutex_pool;
	double sigma_trans;
	EMKalman* emkf;
}pthread_param_emkf;

#endif

