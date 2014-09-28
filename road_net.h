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
 *	FILE road_net.h: Road network processing
**/

#ifndef ROADNET_H
#define ROADNET_H

/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <iostream>
#include <pqxx/pqxx>
#include <queue>

typedef struct t_segment
{
	double latit_begin;
	double latit_end;
	double longit_begin;
	double longit_end;
	unsigned int p_begin;
	unsigned int p_end;
	bool double_way;
}segment;

typedef struct t_point
{
	double latit;
	double longit;
}point;

point* new_point(const double latit, const double longit);

segment* new_segment(const double latit_begin, const double latit_end, const double longit_begin, 
	const double longit_end, const unsigned int p_begin, const unsigned int p_end,
	const bool double_way);

/**
 * Segment index
**/
class SegIndex
{
	public:
		SegIndex(){};
		
		virtual ~SegIndex(){};

		virtual const bool insert(const segment* seg, const unsigned int id)
		{
			return false;
		}
		
		virtual const unsigned int k_nearest(std::list<unsigned int>& ids, 
			const double latit, const double longit, unsigned int k) const
		{
			return 0;
		}
		
		virtual const unsigned int within_distance(std::list<unsigned int>& ids, 
			const double latit, const double longit, 
			const double distance) const 
		{
			return 0;
		}
		
		virtual const double distance_points(const double latit_one,  
			const double latit_two, const double longit_one,  
			const double longit_two)
		{
			return 0;
		}
};

/**
 * PostGis index
**/
class PostGisIndex: public SegIndex
{
	public:
		PostGisIndex();
		
		~PostGisIndex();
		
		const bool insert(const segment* seg, const unsigned int id);
		
		const unsigned int k_nearest(std::list<unsigned int>& ids, 
			const double latit, const double longit, unsigned int k) const;
		
		const unsigned int within_distance(std::list<unsigned int>& ids, 
			const double latit, const double longit, 
			const double distance) const;

		const double distance_points(const double latit_one,  
			const double latit_two, const double longit_one,  
			const double longit_two);
	private:
		pqxx::connection* conn;
		static const std::string database_name;
		static const std::string table_name;
		static const std::string host;
		static const std::string port;
		static const std::string user;
		static const std::string password;
		static const std::string spatial_ref;
		static const std::string srid;

		const bool connect();
		void disconnect();
		const bool create_table();
		void drop_table();
};

/**
 * Road network
**/
class RoadNet
{
	public:
		/**
		  * RoadNet constructor:
		  * @param input_file_name file with road segments
		  * @return
		  * @throws
		  **/
		RoadNet(const std::string& input_file_name);
		
		virtual ~RoadNet();

		const unsigned int segments_within_distance(std::list<unsigned int>& ids, 
			const double latit, const double longit, 
			const double distance) const;

		const double match_segment(const double latit_first,
			const double latit_second, const double longit_first,
			const double longit_second) const;
		
		const double match_segment(const double latit, const double longit,
			const unsigned int prev_seg, const double max_distance) const;

		const double shortest_path(std::list<unsigned int>& short_path, 
			const unsigned int start_seg, const unsigned int end_seg, 
			const double threshold) const;
		
		/*inlines*/
		inline const std::string seg_name(const unsigned int& seg) const
		{
			return seg_names.at(seg);
		}
		
		inline const double seg_latit_begin(const unsigned int& seg) const
		{
			return segments.at(seg)->latit_begin;
		}
		
		inline const double seg_latit_end(const unsigned int& seg) const
		{
			return segments.at(seg)->latit_end;
		}
		
		inline const double seg_longit_begin(const unsigned int& seg) const
		{
			return segments.at(seg)->longit_begin;
		}
		
		inline const double seg_longit_end(const unsigned int& seg) const
		{
			return segments.at(seg)->longit_end;
		}
		
		inline const double segment_length(const unsigned int& seg) const
		{
			//FIXME: length shold be a field of segment
			return lengths.at(seg);
		}
	private:
		std::vector<std::list<unsigned int>*> adj_list;
		std::vector<std::string> seg_names;
		std::map<std::string, unsigned int> seg_ids;
		std::vector<segment*> segments;
		std::vector<double> lengths;
		unsigned int seg_id;
		unsigned int n_segments;
		SegIndex* seg_index;
		double length_longest_segment;
		static const unsigned int num_neigh_match;
		static const unsigned int max_length_segment;

		void add_segment(const std::string id, const double latit_begin, const double latit_end, 
			const double longit_begin, const double longit_end, const unsigned int p_begin, 
			const unsigned int p_end, const bool double_way);
		
		const bool read_road_net(const std::string& input_file_name) throw (std::ios_base::failure);
		void build_adjacency_list();
		void index_segments();
		const double shortest_path(const unsigned int s1, 
			const unsigned int s2, const double threshold) const;
		const bool double_way(const unsigned int seg) const;
		const unsigned int flip_segment(const unsigned int seg) const;
		void compute_segment_lengths();
};

#endif
