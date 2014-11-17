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
#include <math.h>

/**
 * Segment
**/
typedef struct t_segment
{
	double latit_begin;
	double latit_end;
	double longit_begin;
	double longit_end;
	
	double proj_latit_begin;
	double proj_latit_end;
	double proj_longit_begin;
	double proj_longit_end;
	
	unsigned int p_begin;
	unsigned int p_end;
	bool double_way;
	double length;
	std::string name;
} segment;

/**
 * Point
**/
typedef struct t_point
{
	double latit;
	double longit;
}point;

/**
 * Creates new point
**/
point* new_point(const double latit, const double longit);

/**
 * Creates new segment
**/
segment* new_segment
	(
		const double latit_begin, 
		const double latit_end, 
		const double longit_begin, 
		const double longit_end, 
		const double proj_latit_begin, 
		const double proj_latit_end, 
		const double proj_longit_begin, 
		const double proj_longit_end,
		const unsigned int p_begin, 
		const unsigned int p_end,
		const bool double_way
	);

const double std_distance(const double x1, const double x2, const double y1, const double y2);

/**
 * Interface for segment index. 
 * In case you need to replace PostGIS,
 * you have to implement these functionalities.
**/
class SegIndex
{
	public:
		/**
		 * Constructor
		**/
		SegIndex(){};
		
		/**
		 * Destructor
		**/
		virtual ~SegIndex(){};

		/**
		 * Inserts road segment.
		 * @param seg segment
		 * @param id segment identifier
		 * @return true if success
		**/
		virtual const bool insert
			(
				const segment* seg, 
				const unsigned int id
			)
		{
			return false;
		}
		
		/**
		 * Finds k-nearest neighbor segments to 
		 * a point.
		 * @param ids segment ids to be filled
		 * @param latit point latitude
		 * @param longit point longitude
		 * @param k number of neighbors
		 * @return number of neighbors found
		**/
		virtual const unsigned int k_nearest
			(
				std::list<unsigned int>& ids, 
				const double latit, 
				const double longit, 
				unsigned int k
			) 
			const
		{
			return 0;
		}
		

		/**
		 * Finds segments within certain distance from
		 * a point.
		 * @param ids segment ids to be filled
		 * @param latit point latitude
		 * @param longit point longitude
		 * @param distance distance
		 * @return number of segments found
		**/
		virtual const unsigned int within_distance
			(
				std::list<unsigned int>& ids, 
				const double latit, 
				const double longit, 
				const double distance
			) 
			const 
		{
			return 0;
		}
		
		/**
		 * Computes distance between two points.
		 * @param latit_one latitude first point
		 * @param latit_two latitude point two
		 * @param longit_one longitude point one
		 * @param longit_two longitude point two
		 * @return distance
		**/
		virtual const double distance_points
			(
				const double latit_one,  
				const double latit_two, 
				const double longit_one,  
				const double longit_two
			)
		{
			return 0;
		}
		
		/**
		 * Creates new database.
		**/
		virtual const bool create()
		{
			return false;
		}
		
		/**
		 * Projects lat-long points into a planar representation.
		 * @param latit latitude
		 * @param longit longitude
		 * @param x projected latitude
		 * @param y projected longitude
		 * @return true if success
		**/
		virtual const bool project
			(
				const double latit, 
				const double longit, 
				double& x, 
				double& y
			)
		{
			return false;
		}
		
		/**
		 * Drops database.
		**/
		virtual void drop(){};
		
		/**
		 * Given a point p and a segment s, computes the
		 * closest point to p in s.
		 * @param seg segment
		 * @param latit_p latitude point p
		 * @param longit_p longitude point p
		 * @param latit_s latitude point in s
		 * @param longit_s longitude point in s
		 * @return true if success
		**/
		virtual const bool closest_point_segment
			(
				const unsigned int seg, 
				const double latit_p, 
				const double longit_p,
				double& latit_s, 
				double& longit_s
			) 
			const
		{
			return false;
		}
		
		/**
		 * Computes the distance between a point and a segment.
		 * @param seg segment
		 * @param latit latitude point
		 * @param longit longitude point
		 * @return distance
		**/
		virtual const double distance_point_segment
			(
				const unsigned int seg,
				const double latit, 
				const double longit
			) 
			const
		{
			return 0;
		}
};

/**
 * PostGis index. Implements the index interface using PostGis.
 * http://postgis.net/
**/
class PostGisIndex: public SegIndex
{
	public:
		PostGisIndex();
		
		~PostGisIndex();
		
		const bool insert
			(
				const segment* seg, 
				const unsigned int id
			);
		
		const unsigned int k_nearest
			(
				std::list<unsigned int>& ids, 
				const double latit, 
				const double longit, 
				unsigned int k
			) 
				const;
		
		const unsigned int within_distance
			(
				std::list<unsigned int>& ids, 
				const double latit, 
				const double longit, 
				const double distance
			) 
				const;

		const double distance_points
			(
				const double latit_one,  
				const double latit_two, 
				const double longit_one,  
				const double longit_two
			);

		const bool create();
		
		void drop();

		const bool project
			(
				const double latit, 
				const double longit, 
				double& x, 
				double& y
			);
		
		const bool closest_point_segment
			(
				const unsigned int seg, 
				const double latit_p, 
				const double longit_p,
				double& latit_s, 
				double& longit_s
			) 
				const;

		const double distance_point_segment
			(
				const unsigned int seg,
				const double latit, 
				const double longit
			)
				const;
	private:
		//Database connection
		pqxx::connection* conn;

		//PostGis database name, table name, 
		//host, port, user and password. 
		static const std::string database_name;
		static const std::string table_name;
		static const std::string host;
		static const std::string port;
		static const std::string user;
		static const std::string password;
		
		//Spatial reference and srid
		//spatial reference is for the 
		//input representation (lat,long)
		//and we are using EPSG:4326
		//http://en.wikipedia.org/wiki/World_Geodetic_System
		//http://spatialreference.org/ref/epsg/wgs-84/
		//srid is applied for projecting points into a planar
		//representation and we are applying EPSG 26943
		//for SF area. http://spatialreference.org/ref/epsg/26943/
		static const std::string spatial_ref;
		static const std::string srid;

		const bool connect();
		void disconnect();
		const bool create_table();
		void drop_table();
};

/**
 * Road network representation. Relies on a spatial index
 * for segments and implements several functionalities
 * such as shortest paths in the road structure. Some
 * functionalities are direct calls to the segment index interface
 * but we try to hide the segment index behind this class.
**/
class RoadNet
{
	public:
		/*CONSTRUCTORS*/

		/**
		 * Reads the road network from an input file.
		 * There is another constructor that can generate
		 * this input file given the list of segments
		 * and their coordinates, but it takes longer.
		 * @param input_file_name input file
		**/
		RoadNet(const std::string& input_file_name);
		
		/**
		 * Creates the road network and writes it into a 
		 * file.
		 * @param input_file_name input file with segment data
		 * @param output_file_name output file
		**/
		RoadNet
			(
				const std::string& input_file_name, 
				const std::string& output_file_name
			);
		
		/**
		 * Destructor.
		**/
		virtual ~RoadNet();

		/**
		 * Finds segments within certain distance from
		 * a point.
		 * @param ids segment ids to be filled
		 * @param latit point latitude
		 * @param longit point longitude
		 * @param distance distance
		 * @return number of segments found
		**/
		const unsigned int segments_within_distance
			(
				std::list<unsigned int>& ids, 
				const double latit, 
				const double longit, 
				const double distance
			) 
				const;

		/**
		 * Finds the closest segment to a point.
		 * @param latit point latitude
		 * @param longit point longitude
		 * @return segment identifier
		**/
		const unsigned int closest_segment
			(
				const double latit, 
				const double longit
			) 
				const;
		
		const bool closest_point_segment(const unsigned int seg,
			const double latit_p, const double longit_p, 
			double& latit_s, double& longit_s) const;
		
		/**
		 * Computes the distance of the shortest path between 
		 * two segments given the points over the segments.
		 * @param s1 start segment
		 * @param s2 end segment
		 * @param latit_from latitude point start segment
		 * @param latit_to latitude point end segment
		 * @param longit_from longitude point start segment
		 * @param longit_to longitude point end segment
		 * @return distance
		**/
		const double shortest_path(const unsigned int s1, 
			const unsigned int s2, const double latit_from,
			const double latit_to, const double longit_from,
			const double longit_to) const;
		
		/**
		 * Computes the distance of the shortest path between 
		 * two segments given the points over the segments.
		 * @param shortest_path sequence of segments in the path
		 * @param s1 start segment
		 * @param s2 end segment
		 * @param latit_from latitude point start segment
		 * @param latit_to latitude point end segment
		 * @param longit_from longitude point start segment
		 * @param longit_to longitude point end segment
		 * @return distance
		**/
		const double shortest_path(
			std::list<unsigned int>& shortest_path, const unsigned int s1, 
			const unsigned int s2, const double latit_from,
			const double latit_to, const double longit_from,
			const double longit_to) const;
		
		/**
		 * Computes the distance between a point and a segment.
		 * @param seg segment
		 * @param latit latitude point
		 * @param longit longitude point
		 * @return distance
		**/
		const double distance_point_segment
			(
				const unsigned int seg,
				const double latit, 
				const double longit
			) 
				const;
		
		/*INLINES*/

		/**
		 * Gets the name of a segment as given in the input file.
		 * If a segment is in both ways, we create a "SEG" and 
		 * "-SEG" name, where SEG is the name in the input file.
		 * @param seg segment identifier
		 * @return segment name
		**/
		inline const std::string seg_name(const unsigned int& seg) const
		{
			return segments.at(seg)->name;
		}
		
		/**
		 * Gets an integer id of a segment given its name. 
		 * Most of the time, segments are manipulated through these
		 * integer ids instead of their string names.
		 * @param name name of segment
		 * @return segment id
		**/
		inline const unsigned int seg_ID(const std::string& name) const
		{
			return seg_ids.at(name);
		}
		
		/**
		 * Gets the latitude of the starting point of a segment.
		 * @param seg segment
		 * @return latitude
		**/
		inline const double seg_latit_begin(const unsigned int& seg) const
		{
			return segments.at(seg)->latit_begin;
		}
		
		/**
		 * Gets the latitude of the end point of a segment.
		 * @param seg segment
		 * @return latitude
		**/
		inline const double seg_latit_end(const unsigned int& seg) const
		{
			return segments.at(seg)->latit_end;
		}
		
		/**
		 * Gets the longitude of the starting point of a segment.
		 * @param seg segment
		 * @return longitude
		**/
		inline const double seg_longit_begin(const unsigned int& seg) const
		{
			return segments.at(seg)->longit_begin;
		}
		
		/**
		 * Gets the longitude of the end point of a segment.
		 * @param seg segment
		 * @return longitude
		**/
		inline const double seg_longit_end(const unsigned int& seg) const
		{
			return segments.at(seg)->longit_end;
		}
		
		/**
		 * Gets the length of a segment in meters.
		 * @param seg segment
		 * @return length
		**/
		inline const double segment_length(const unsigned int& seg) const
		{
			return segments.at(seg)->length;
		}
		
		/**
		 * Projects lat-long points into a planar representation.
		 * @param latit latitude
		 * @param longit longitude
		 * @param x projected latitude
		 * @param y projected longitude
		 * @return true if success
		**/
		inline const bool project
			(
				const double latit, 
				const double longit, 
				double& x, 
				double& y
			) 
				const
		{
			return seg_index->project(latit, longit, x, y);
		}
		
		/**
		 * Gets a pointer to the actual segment given its ID.
		 * @param seg segment id
		 * @return segment
		**/
		inline segment* get_segment(const unsigned int& seg) const
		{
			return segments.at(seg);
		}

		/**
		 * Computes distance between two points.
		 * @param latit_one latitude first point
		 * @param latit_two latitude point two
		 * @param longit_one longitude point one
		 * @param longit_two longitude point two
		 * @return distance
		**/
		inline const double distance_points
			(
				const double latit_one,  
				const double latit_two, 
				const double longit_one,  
				const double longit_two
			) 
				const 
		{
			return seg_index->distance_points(latit_one, 
				latit_two, longit_one, longit_two);
		}
		
		/**
		 * Gets the size of the road network in number of segments.
		 * @return size
		**/
		inline const unsigned int size() const
		{
			return segments.size();
		}
		
		inline void clear_saved_shortest_paths(const unsigned int seg)
		{
			saved_shortest_paths[seg]->clear();
		}
	private:
		/*VARIABLES*/
		std::vector<std::list<unsigned int>*> adj_list;
		std::map<std::string, unsigned int> seg_ids;
		std::vector<segment*> segments;
		unsigned int seg_id;
		unsigned int n_segments;
		SegIndex* seg_index;
		double length_longest_segment;
		std::vector< std::map<unsigned int, double> * > saved_shortest_paths; 
		
		/*METHODS*/

		void add_segment
			(
				const std::string id, 
				const double latit_begin, 
				const double latit_end, 
				const double longit_begin, 
				const double longit_end, 
				const unsigned int p_begin, 
				const unsigned int p_end, 
				const bool double_way
			);
		
		void build_adjacency_list();
		void index_segments();
		const bool double_way(const unsigned int seg) const;
		const unsigned int flip_segment(const unsigned int seg) const;
		void compute_segment_lengths();
		void project_segments();
		
		const bool create
			(
				const std::string& input_file_name,
				const std::string& output_file_name
			) 
				throw (std::ios_base::failure);
		
		const bool write(const std::string& output_file_name) 
			throw (std::ios_base::failure);
		
		const bool build(const std::string& input_file_name) 
		 	throw (std::ios_base::failure);
		
		const bool read(const std::string& input_file_name)
			throw (std::ios_base::failure);
};

#endif
