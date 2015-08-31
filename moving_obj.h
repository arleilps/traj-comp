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
 *	FILE moving_obj.h: Moving object processing
**/

#ifndef MOVINGOBJ_H
#define MOVINGOBJ_H

/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <iostream>
#include <pthread.h>

#include "road_net.h"
#include "perf.h"

/**
 * GPS update
**/
typedef struct t_update
{
	double latit;
	double longit;
	double proj_latit;
	double proj_longit;
	std::string object;
	unsigned int time;
} update;

typedef struct t_em_update_info
{
	double dist;
	double total_dist;
	unsigned int time;
	unsigned int total_time;
	double sigma;
} em_update_info;

em_update_info* new_em_update_info
	(
		const double dist,
		const double total_dist,
		const unsigned int time,
		const unsigned int total_time,
		const double sigma
	);

/**
 * Creates new GPS update.
**/
update* new_update
	(
		const std::string obj, 
		const double latit, 
		const double longit, 
		unsigned int time
	);

/**
 * Creates new GPS update.
**/
update* new_update
	(
		const std::string obj, 
		const double latit, 
		const double longit, 
		const double proj_latit, 
		const double proj_longit, 
		unsigned int time
	);

/**
 * Deletes updates in a list.
**/
void delete_list_updates(std::list<update*>* updates);

/**
 * Map-matched update (segment,time)
**/
typedef struct t_seg_time
{
	unsigned int segment;
	unsigned int time;
	double dist;
	const update* up;
	unsigned int id;
} seg_time;

/**
 * Creates new map-matched trajectory update.
**/
seg_time* new_seg_time
	(
		const unsigned int segment, 
		const unsigned int time,
		double dist,
		const update* up=NULL
	);

typedef struct t_p_thread_param_map
{
	std::vector<std::list< update* > * >* updates;
	unsigned int* pointer;
	pthread_mutex_t* mutex_pool;
	pthread_mutex_t* mutex_file;
	std::ofstream* output_file;
	RoadNet* net;
}p_thread_param_map;

typedef struct t_dist_time
{
	unsigned int time;
	double dist;
} dist_time;

dist_time* new_dist_time(const double dist, const unsigned int time);

/**
 * Implements functionalities for trajectory manipulation (e.g. creation, map-matching etc.)
**/
class Trajectory
{
	public:
		/*CONSTRUCTORS*/

		/**
		 * Creates an empty trajectory
		 * @param obj id of the object associated with the trajectory (optional)
		**/
		Trajectory(const std::string obj="");

		/**
		 * Copy constructor
		**/
		Trajectory(const Trajectory& traj);
		
		/**
		 * Destructor
		**/
		virtual ~Trajectory();
		
		 /*STATIC METHODS*/
		 
		 /**
		  * Performs map-matching using Hidden markov model and viterbi algorithm.
		  * Based on paper: An HMM-based Map Matching Method with Cumulative 
		  * Proximity-Weight Formulation
		  * @param updates gps updates (lat-long) to be matched
		  * @param net road network structure
		  * @return map-matched trajectory
		 **/
		static Trajectory* map_matching
			(
				const std::list<update*>& updates, 
				RoadNet* net
			);
		
		/**
		 * Reads gps updates from an input file. See README for formatting.
		 * @param updates updates list to be filled
		 * @param input_file_name input file
		 * @param net road network structure
		 * @return number of different objects for which the updates were read
		 * @throws ios_base::failure if has problems opening or reading file
		**/
		static const unsigned int read_updates
			(
				std::vector<std::list< update* > * >& updates, 
				const std::string input_file_name, 
				const RoadNet* net
			) 
				throw (std::ios_base::failure);
		/**
		 * Deletes trajectories from a list.
		 * @param trajectories list of trajectories to be deleted
		**/
		static void delete_trajectories
			(
				std::list<Trajectory*>* trajectories
			);
		
		/**
		 * Reads map-matched trajectories from an input file.
		 * We assume that this program generated the trajectories
		 * so the input-output formats should be compatible.
		 * @param trajectories trajectories list to be filled
		 * @param input_file_name input file
		 * @param net road network structure
		**/
		static const unsigned int read_trajectories
			(
				std::list< Trajectory * >& trajectories, 
				const std::string input_file_name, 
				const RoadNet* net
			) 
				throw (std::ios_base::failure);

		static void break_trajectories
			(
				std::list< Trajectory * >& trajectories,
				RoadNet* net
			);

		static void expand_trajectories
			(
				std::list< Trajectory * >& trajectories,
				RoadNet* net
			);
		/**
		 * Map-matches trajectories from an input file and writes
		 * them into an output file.
		 * @param input_file_name input file with gps updates
		 * @param output_file_name output file 
		 * @param net road network structure
		 * @param _sigma standard deviation of the gps data
		 * @param _radius radius*sigma distance to be checked for matching segments
		 * @param _beta_const constant beta for matching (see paper)
		 * @param _time_div another constant for matching (see paper)
		 * @return true in case of success, false otherwise
		**/
		static const bool write_map_matched_trajectories
			(
				const std::string input_file_name,
				const std::string output_file_name,
				RoadNet* net,
				const double _sigma=SIGMA,
				const double _radius=RADIUS,
				const double _beta_const=BETACONST,
				const double _time_div=TIMEDIV
			);
		
		static const bool write_map_matched_trajectories_multithreads
			(
				const std::string input_file_name,
				const std::string output_file_name,
				RoadNet* net,
				const double _sigma=SIGMA,
				const double _radius=RADIUS,
				const double _beta_const=BETACONST,
				const double _time_div=TIMEDIV
			);

		static void delete_dist_times(std::list < dist_time* >& dist_times);

		 /*OBJECT METHODS*/

		void get_dist_times_uniform
			(
				std::list < dist_time* >& dist_times,
				RoadNet* net
			) 
				const;
		
		void set_times_uniform(RoadNet* net);

		void get_em_rep
			(
				std::vector < std::vector< std::pair< unsigned int, em_update_info* > * > * >
					& updates_em,
				const double sigma_gps, 
				RoadNet* net
			)
				const;

		/**
		 * Adds a map-matched update to the trajectory
		 * @param segment 
		 * @param start_time time when object started segment
		 * @param end_time time when object ended segment
		 * @param update update that was matched (optional)
		**/
		virtual void add_update
			(
				const unsigned int segment, 
				const unsigned int time, 
				const double dist, 
				const update* up=NULL
			);
		
		void add_update_front
			(
				const unsigned int segment, 
				const unsigned int time, 
				const double dist, 
				const update* up=NULL
			);
		
		/**
		 * Writes a trajectory into a file.
		 * Format <object, segment, timestamp>, one per line.
		 * @param output_file output file
		 * @param net road network structure
		**/
		void write
			(
				std::ofstream& output_file, 
				const RoadNet* net
			) const;

		/**
		 * Prints a trajectory on the screen (for testing).
		**/
		void print()  const;
		
		/**
		 * Extends a trajectory assuming objects move in 
		 * shortest paths.
		 * @param net road network structure
		**/
		void extend_traj_shortest_paths(const RoadNet* net);

		/**
		 * Removes repeated segments from a trajectory.
		**/
		void remove_repeated_segments();

		void decompose_online(std::list<Trajectory*>& decomp) const;

		void decompose_delay
			(
				std::list<Trajectory*>& decomp, 
				const unsigned int delay,
				RoadNet* net
			) const;
		
		void append(Trajectory* traj);

		void break_trajectory
			(
				std::list< Trajectory * >& trajectories,
				RoadNet* net
			);
		
		/*PUBLIC VARIABLES*/
		
		//iterator for trajectories
		typedef std::list< seg_time* >::iterator iterator;
		
		typedef std::list< seg_time* >::const_iterator const_iterator;
		
		/*INLINES*/

		/**
		 * Iterator to the beginning of the trajectory
		**/
		inline iterator begin()
		{
			return seg_time_lst.begin();
		}
		
		/**
		 * Iterator to the end of the trajectory
		**/
		inline iterator end() 
		{
			return seg_time_lst.end();
		}
		
		/**
		 * Iterator to the last segment of the trajectory
		**/
		inline seg_time* back()
		{
			return seg_time_lst.back();
		}
		
		inline seg_time* front()
		{
			return seg_time_lst.front();
		}
		
		/**
		 * Gets the size of a trajectory
		**/
		inline const unsigned int size()
		{
			return size_traj;
		}
		
		/**
		 * Gets the object associated with the trajectory
		**/
		inline const std::string object()
		{
			return obj;
		}

		static inline void set_num_threads(const unsigned int _num_threads)
		{
			num_threads = _num_threads;
		}

		static const inline double max_speed()
		{
			return MAXSPEED;
		}

		static const inline double min_speed()
		{
			return MINSPEED;
		}
			
	private:
		/*OBJECT VARIABLES*/

		std::list< seg_time* > seg_time_lst;
		unsigned int size_traj;
		double prob;
		std::string obj;
		
		/*STATIC VARIABLES*/

		static double time_div;
		static double beta_const;
		static double sigma;
		static double radius;
		static unsigned int max_cand_matches;
		static unsigned int num_threads;
		
		/*CONSTANTS*/

		static const double TIMEDIV;
		static const double BETACONST;
		static const double SIGMA;
		static const double RADIUS;
		static const double MAXSPEED;
		static const double MINSPEED;
		static const unsigned int MAXCANDMATCHES;
		static const double MAXLENGTHSHORTESTPATH;
		
		/*OBJECT METHODS*/

		static const double transition_prob
			(
				const unsigned int seg_from, 
				const unsigned int seg_to,
				const double latit_from, 
				const double latitt_to,
				const double longit_from, 
				const double longit_to,
				const unsigned int time_from, 
				const unsigned int time_to,
				const double gcd,
				const RoadNet* net
			);
		
		static void cand_seg_probs
			(
				std::vector < std::pair < unsigned int, double > * >& seg_probs, 
				const update* up, const RoadNet* net
			);
};

typedef struct t_p_thread_param_exp
{
	std::list < Trajectory* >* trajectories;
	std::list < Trajectory* >::iterator* pointer;
	pthread_mutex_t* mutex_pool;
	RoadNet* net;
}p_thread_param_exp;

/**
 * Generic class for managing trajectory data using a database.
**/
class TrajDBStorage
{
	public:
		/*Constructor*/
		TrajDBStorage()
		{
			n_updates = 0;
		}
		
		/*Destructor*/
		virtual ~TrajDBStorage(){};
		
		/*Creates table*/
		virtual const bool create()
		{
			return false;
		}
		
		/*Drops table*/
		virtual const bool drop()
		{
			return false;
		}

		/**
		 * Inserts a trajectory update.
		 * @param obj object id
		 * @param st seg_time object for the update
		**/
		virtual const bool insert
			(
				const std::string& obj,
				const seg_time& st 
			)
		{
			return false;
		}
		
		/**
		 * Recovers objects that have a given segment in a given time range.
		 * @param segment segment id
		 * @param objs list of objects to be updated
		 * @param time_begin start time of the range
		 * @param time_end end time of the range
		**/
		virtual const bool query_segment_time
			(
				const unsigned int segment,
				std::list<std::string>& objs,
				const unsigned int time_begin=0,
				const unsigned int time_end=0
			)
				const
		{
			return false;
		}
		
		virtual seg_time* where_at(const std::string& obj, 
			const unsigned int time)
			const
		{
			return NULL;
		}
		
		virtual Trajectory* get_traj
			(
				const std::string& obj,
				const unsigned int time=0
			) const
		{
			return NULL;
		}

		/*Returns number of udpates made to the database*/
		const unsigned int updates() const
		{
			return n_updates;
		}
	protected:
		unsigned int n_updates;
};

/**
 * Implements a trajectory database as a PostGis table.
**/
class TrajDBPostGis: public TrajDBStorage
{
	public:
		/*Constructor*/
		TrajDBPostGis():TrajDBStorage()
		{
			connect();
		}
		
		/*Destructor*/
		virtual ~TrajDBPostGis(){};
		
		const bool create();
		
		const bool drop();

		const bool insert
			(
				const std::string& obj,
				const seg_time& st 
			);
		
		virtual seg_time* where_at(const std::string& obj, const unsigned int time)
			const;
		
		virtual Trajectory* get_traj
			(
				const std::string& obj,
				const unsigned int time=0
			) const;
		
		static void set_config(const std::string& input_file_name);
	protected:
		//PostGis database name, table name, 
		//host, port, user and password.
		static std::string database_name;
		static const std::string table_name;
		static std::string host;
		static std::string port;
		static std::string user;
		static std::string password;
		
		//Spatial reference and srid
		//spatial reference is for the 
		//input representation (lat,long)
		//and we are using EPSG:4326
		//http://en.wikipedia.org/wiki/World_Geodetic_System
		//http://spatialreference.org/ref/epsg/wgs-84/
		//srid is applied for projecting points into a planar
		//representation and we are applying EPSG 26943
		//for SF area. http://spatialreference.org/ref/epsg/26943/)
		static std::string srid;
		static std::string spatial_ref;

		//Database connection
		pqxx::connection* conn;

		const bool connect();
};

typedef struct t_query
{
	std::string obj;
	unsigned int time;
}query;

/**
 * Implements several functionalities (insert, query etc.) for managing a trajectory database.
**/
class TrajDB
{
	public:
		/**
		 * Constructor.
		 * @param _net road network
		**/
		TrajDB(RoadNet* _net)
		{
			net = _net;
			db = new TrajDBPostGis();
			_num_updates_orig = 0;
			_num_updates_inserted = 0;
			_num_traj_inserted = 0;
			_num_queries = 0;
			query_t = new ExecTime();
			insert_t = new ExecTime();
		}
		
		/* Destructor */
		virtual ~TrajDB()
		{
			delete db;
			delete query_t;
			delete insert_t;
		};
		
		/*Creates a new database*/
		const bool create()
		{
			return db->create();
		}
		
		/*Drops the current database*/
		const bool drop()
		{
			return db->drop();
		}

		/**
		 * Inserts all trajectories in a map-matched trajectory file into 
		 * the database.
		 * @param input_file_name input file name
		 * @return true in case of success, false otherwise
		**/
		virtual const bool insert(const std::string& input_file_name);
		
		/**
		 * Inserts a trajectory into the database.
		 * @param obj id of the object associated with the trajectory
		 * @param traj trajectory
		 * @return true in case of success, false otherwise
		**/
		virtual const bool insert(const std::string& obj, Trajectory& traj);
		
		/**
		 * Inserts a trajectory update for a given object into the database.
		 * @param obj object id
		 * @param st seg_time update
		**/
		const bool insert
			(
				const std::string& obj,
				const seg_time& st
			);
		
		virtual seg_time* where_at
			(
				const std::string& obj,
				const unsigned int time
			) const;

		void where_at
			(
				const std::string& query_file_name,
				const std::string& output_file_name
			);
		
		virtual void where_at_part
			(
				const std::string& query_file_name,
				const std::string& output_file_name
			)
		{
			return where_at(query_file_name, output_file_name);
		}

		 void read_queries
		 	(
		 		std::list<query*>& queries,
		 		const std::string query_file_name
		 	) const;
		
		virtual void train(const std::string& training_traj_file_name){};

		void write_query_results
			(
				const std::list<query*>& queries,
				const std::list<seg_time*>& results,
				const std::string& output_file_name
			) const;

		/*INLINES*/
		
		/*Statistics*/
		inline double inserts_second() const
		{
			return (double)  _num_updates_orig / _insert_time;
		}

		inline double queries_second() const
		{
			return (double) _num_queries / _query_time;
		}

		inline unsigned int num_queries() const
		{
			return _num_queries;
		}

		inline double query_time() const
		{
			return _query_time;
		}
		
		inline double insert_time() const
		{
			return _insert_time;
		}

		virtual double training_time() const
		{
			return 0;
		}

		inline unsigned int num_updates_orig() const
		{
			return _num_updates_orig;
		}

		inline unsigned int num_updates_inserted() const
		{
			return _num_updates_inserted;
		}

		virtual unsigned int num_updates_train() const
		{
			return 0;
		}

		inline unsigned int num_traj_inserted() const
		{
			return _num_traj_inserted;
		}

		virtual unsigned int num_traj_train() const
		{
			return 0;
		}

		inline double compression_ratio() const
		{
			return (double) _num_updates_orig / _num_updates_inserted;
		}
	protected:
		RoadNet* net;
		TrajDBStorage* db;
		double _query_time;
		double _insert_time;
		unsigned int _num_updates_orig;
		unsigned int _num_updates_inserted;
		unsigned int _num_traj_inserted;
		unsigned int _num_queries;
		double _compression_ratio;
		ExecTime* query_t;
		ExecTime* insert_t;
};

#endif
