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
 *	FILE road_net.cc: Road network processing
**/


/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <pqxx/pqxx>
#include <queue>
#include <boost/heap/fibonacci_heap.hpp>

/*my includes*/
#include "road_net.h"
#include "io.h"

const std::string PostGisIndex::database_name = "test";
const std::string PostGisIndex::table_name = "seg_index";
const std::string PostGisIndex::host = "127.0.0.1";
const std::string PostGisIndex::port = "5432";
const std::string PostGisIndex::user = "traj_comp";
const std::string PostGisIndex::password="traj_comp";
const std::string PostGisIndex::srid = "26943";
const std::string PostGisIndex::spatial_ref = "4326";
const unsigned int RoadNet::num_neigh_match = 20;

/**
 * new_point: Creates a new point with the parameters
**/
point* new_point(const double latit, const double longit)
{
        point* p = new point;
        p->latit = latit;
        p->longit = longit;

        return p;
}

/**
 * new_segment: Creates a new segment with the parameters
**/
segment* new_segment(const double latit_begin, const double latit_end, 
	const double longit_begin, const double longit_end,
	const unsigned int p_begin, const unsigned int p_end,
	const bool double_way, const std::string& name)
{
	segment* seg = new segment;
	seg->latit_begin = latit_begin;
	seg->latit_end = latit_end;
	seg->longit_begin = longit_begin;
	seg->longit_end = longit_end;	
	seg->p_begin = p_begin;
	seg->p_end = p_end;
	seg->double_way = double_way;
	seg->name = name;

	return seg;
}

/**
 * RoadNet constructor:
 * @param input_file_name file with road segments
 * @return
 * @throws io_base::failure
 **/
RoadNet::RoadNet(const std::string& input_file_name)
{
	seg_id = 0;
	n_segments = 0;
	seg_index = new PostGisIndex();
	read(input_file_name);
}

/**
 * RoadNet constructor:
 * @param input_file_name file with road segments
 * @return
 * @throws io_base::failure
 **/
RoadNet::RoadNet(const std::string& input_file_name,
	const std::string& output_file_name)
{
	seg_id = 0;
	n_segments = 0;
	seg_index = new PostGisIndex();
	create(input_file_name, output_file_name);
}

const bool RoadNet::read(const std::string& input_file_name)
	throw (std::ios_base::failure)
{
	std::ifstream road_net_file(input_file_name.c_str(), std::ios::in);
       	std::string line_str;
       	std::vector< std:: string > line_vec;

        if(! road_net_file.is_open())
       	{
        	std::cerr << "Error: Could not open road network file" 
			<< input_file_name << std::endl << std::endl;
		
		return false;
        }       

	std::getline(road_net_file, line_str);
	line_vec = split(line_str,',');

	//First line should contain the number of segments
	if(line_vec.size() != 1)
	{
		std::cerr << "Error: Invalid road network file format, check the README file -- " 
			<< input_file_name << std::endl << std::endl;
		road_net_file.close();
		
		return false;
	}
	else
	{
		 n_segments = atoi(line_vec[0].c_str());
	}

	segments.reserve(n_segments);
	adj_list.reserve(n_segments);
	length_longest_segment = 0;

	unsigned int s = 0;
	segment* seg;
	int double_way;

	//Reading segments
	while(s < n_segments)
	{
		std::getline(road_net_file, line_str);
		line_vec = split(line_str,',');

		if(line_vec.size() != 12)
		{
			std::cerr << "Error: Invalid road network file format, check the README file -- " 
				<< input_file_name << std::endl << std::endl;
			road_net_file.close();

			return false;
		}
		else
		{
			seg = new segment;
			
			seg->latit_begin = atof(line_vec[1].c_str());
			seg->latit_end = atof(line_vec[2].c_str());
			seg->longit_begin = atof(line_vec[3].c_str());
			seg->longit_end = atof(line_vec[4].c_str());
			
			seg->proj_latit_begin = atof(line_vec[5].c_str());
			seg->proj_latit_end = atof(line_vec[6].c_str());
			seg->proj_longit_begin = atof(line_vec[7].c_str());
			seg->proj_longit_end = atof(line_vec[8].c_str());
			
			//default
			seg->p_begin = 0;
			seg->p_end = 0;

			double_way = atoi(line_vec[9].c_str());

			if(double_way == 0)
			{
				seg->double_way = false;
			}
			else
			{
				seg->double_way = true;
			}

			seg->length = atof(line_vec[10].c_str());
	
			if(seg->length > length_longest_segment)
			{
				length_longest_segment = seg->length;
			}

			seg->name = line_vec[11];

			segments.push_back(seg);

			s++;
		}
	}
	
	std::list<unsigned int>* neighbors;
	std::getline(road_net_file, line_str);

	while(! road_net_file.eof())
	{
		line_vec = split(line_str,',');
		
		neighbors = new std::list<unsigned int>;

		for(unsigned int i = 1; i < line_vec.size(); i++)
		{
			neighbors->push_back(atoi(line_vec[i].c_str()));
		}

		adj_list.push_back(neighbors);
		
		std::getline(road_net_file, line_str);
	}

	road_net_file.close();

	std::cout << segments.size() << " " << adj_list.size() << std::endl;

	return true;
}

/**
 * create: Reads road network data (segments) and 
 * writes road network into output_file
 * @param input_file_name file with road segments
 * @param output_file_name output file
 * @return true if success, false otherwise
 * @throws io_base::failure
 **/
const bool RoadNet::create(const std::string& input_file_name,
	const std::string& output_file_name) throw (std::ios_base::failure)
{
	try
	{
		if(build(input_file_name))
		{
			build_adjacency_list();
			index_segments();
			compute_segment_lengths();
			project_segments();
			write(output_file_name);
		}
		else
		{
			std::cerr << "Error: empty road network" 
				<< input_file_name << std::endl << std::endl;
			return false;
		}
	}
	catch(const std::ios_base::failure &e)
	{
		std::cerr << "Error: Could not read road network file" 
			<< input_file_name 
			<< std::endl << std::endl;
		std::cerr << e.what() << std::endl;

		return false;
	}

	return false;
}

void print_segment(segment* seg, const unsigned int id,
	std::ofstream& road_net_file)
{
	road_net_file << id << "," 
		<< seg->latit_begin << "," 
		<< seg->latit_end << ","
		<< seg->longit_begin << ","
		<< seg->longit_end << ","
		
		<< seg->proj_latit_begin << "," 
		<< seg->proj_latit_end << ","
		<< seg->proj_longit_begin << ","
		<< seg->proj_longit_end << ",";
	
	if(seg->double_way)
	{
		road_net_file << "1,";
	}
	else
	{
		road_net_file << "0,";
	}

	road_net_file << seg->length << ","
		<< seg->name << "\n";
}

const bool RoadNet::write(const std::string& output_file_name) throw (std::ios_base::failure)
{
	std::ofstream road_net_file(output_file_name.c_str(), std::ios::out);

        if(! road_net_file.is_open())
       	{
        	std::cerr << "Error: Could not open road network file" 
			<< output_file_name << std::endl << std::endl;
		
		return false;
        }       
	
	road_net_file << n_segments << "\n";

	for(unsigned int s = 0; s < segments.size(); s++)
	{
		print_segment(segments.at(s), s, road_net_file);
	}
	
	for(unsigned int s = 0; s < adj_list.size(); s++)
	{
		road_net_file << s;

		for(std::list<unsigned int>::iterator it = adj_list.at(s)->begin();
			it != adj_list.at(s)->end(); ++it)
		{
			road_net_file << "," << *it;
		}

		road_net_file << "\n";
	}

	road_net_file.close();

	return true;
}

/**
 * build: Builds the road network from data
 * @param input_file_name file with road segments
 * @return true if success, false otherwise
 * @throws io_base::failure
 **/
const bool RoadNet::build(const std::string& input_file_name) 
	throw (std::ios_base::failure)
{
	std::ifstream road_net_file(input_file_name.c_str(), std::ios::in);
       	std::string line_str;
       	std::vector< std:: string > line_vec;

        if(! road_net_file.is_open())
       	{
        	std::cerr << "Error: Could not open road network file" 
			<< input_file_name << std::endl << std::endl;
		
		return false;
        }       

	std::getline(road_net_file, line_str);
	line_vec = split(line_str,',');

	unsigned int n_points;

	//First line should contain the number of points and segments
	if(line_vec.size() != 2)
	{
		std::cerr << "Error: Invalid road network file format, check the README file -- " 
			<< input_file_name << std::endl << std::endl;
		road_net_file.close();
		
		return false;
	}
	else
	{
		 n_points = atoi(line_vec[0].c_str());
		 n_segments = atoi(line_vec[1].c_str());
	}

	adj_list.reserve(2*n_segments);
	segments.reserve(2*n_segments);

	std::map<std::string, point*> points;
	std::map<std::string, unsigned int> point_ids;
 
	unsigned int p = 0;
	std::string id;
	double latit;
	double longit;

	/*Reading points*/
	while(p < n_points)
	{
		std::getline(road_net_file, line_str);
		line_vec = split(line_str,',');
		
		//Line: 40984,37.7401044,-122.4337842,49449
		if(line_vec.size() < 3)
		{
			std::cerr << "Error: Invalid road network file format, check the README file -- " 
				<< input_file_name << std::endl << std::endl;
			road_net_file.close();

			return false;
		}
		else
		{
			id = line_vec[0];
			latit = atof(line_vec[1].c_str());
			longit = atof(line_vec[2].c_str());

			points[id] = new_point(latit, longit);
			point_ids[id] = p;
		}

		p++;
	}

	unsigned int s = 0;
	std::string p_one;
	std::string p_two;
	std::string dir;	
	
	/*Reading road segments*/
	while(s < n_segments)
	{
		std::getline(road_net_file, line_str);
		line_vec = split(line_str,',');
		
		if(line_vec.size() < 3)
		{
			std::cerr << "Error: Invalid road network file format, check the README file -- " 
				<< input_file_name << std::endl << std::endl;
			road_net_file.close();
               		
			return false;
		}
		else
		{
			id = line_vec[0];
			p_one = line_vec[1];
			p_two = line_vec[2];
			dir = line_vec[3];
			
			/*Zero-length segments are not considered*/
			if(points[p_one]->latit != points[p_two]->latit or points[p_one]->longit != points[p_two]->longit)
			{
				if(dir == "0")
				{
					//Undirected
					add_segment(id, points[p_one]->latit, points[p_two]->latit,
						points[p_one]->longit, points[p_two]->longit, 
						point_ids[p_one], point_ids[p_two], true);
					
					id = "-1"+id; 
		
					add_segment(id, points[p_two]->latit, points[p_one]->latit,
						points[p_two]->longit, points[p_one]->longit,
						point_ids[p_two], point_ids[p_one], true);
				}
				else
				{
					//Directed
					add_segment(id, points[p_one]->latit, points[p_two]->latit,
						points[p_one]->longit, points[p_two]->longit,
						point_ids[p_one], point_ids[p_two], false);
				}
			}
		}

		s++;
	}
	
	n_segments = segments.size();

	for(std::map<std::string, point*>::iterator it = points.begin(); it != points.end(); ++it)
	{
		delete it->second;
	}

	return true;
}

/**
 * Builds an adjacency list for segments
 * FIXME: this could be done faster using an index
**/
void RoadNet::build_adjacency_list()
{
	adj_list.reserve(n_segments);
	
	for(unsigned int s1 = 0; s1 < segments.size(); s1++)
	{
		adj_list.push_back(new std::list<unsigned int>);

		for(unsigned int s2 = 0; s2 < segments.size(); s2++)
		{
			if(s1 != s2)
			{
				if(segments[s1]->p_end == segments[s2]->p_begin)
				{
					adj_list.back()->push_back(s2);	
				}
			}	
		}
	}	
}

/**
 * Destructor
**/
RoadNet::~RoadNet()
{
	delete seg_index;

	for(unsigned int s = 0; s < segments.size(); s++)
	{
		delete segments.at(s);
		delete adj_list.at(s);
	}
}

void RoadNet::add_segment(const std::string id, const double latit_begin, const double latit_end,
	const double longit_begin, const double longit_end, const unsigned int p_begin,
	const unsigned int p_end, const bool double_way)
{
	seg_ids[id] = seg_id;
	segments.push_back(new_segment(latit_begin, latit_end, longit_begin, longit_end, 
		p_begin, p_end, double_way, id));	

	seg_id++;
}

/**
 * Indexes the road segments using seg_index
**/
void RoadNet::index_segments()
{
	seg_index->create();
	
	for(unsigned int s = 0; s < segments.size(); s++)
	{
		seg_index->insert(segments.at(s), s);
	}
}

const bool RoadNet::double_way(const unsigned int seg) const
{
	return segments.at(seg)->double_way;
}

const unsigned int RoadNet::flip_segment(const unsigned int seg) const
{
	std::string flip_id = "-1" + segments.at(seg)->name;
	std::map<std::string, unsigned int>::const_iterator it = seg_ids.find(flip_id);
	return it->second;	
}

const double RoadNet::match_segment(const double latit_first, 
	const double latit_second, const double longit_first,
	const double longit_second) const
{
	std::list<unsigned int> point_close_first;
	seg_index->k_nearest(point_close_first, latit_first, longit_first, 
		num_neigh_match);

	std::list<unsigned int> point_close_second;
	seg_index->k_nearest(point_close_second, latit_second, 
	longit_second, num_neigh_match);

	unsigned int closest_seg = point_close_first.back();
	double closest_dist = (n_segments + 1) * length_longest_segment;
	double threshold = n_segments * length_longest_segment;
	double dist;
	
	for(std::list<unsigned int>::iterator it_first = point_close_first.begin();
		it_first != point_close_first.end(); ++it_first)
	{
		for(std::list<unsigned int>::iterator it_second = point_close_second.begin();
			it_second != point_close_second.end(); ++it_second)
		{
			dist = shortest_path(*it_first, *it_second, threshold);

			if(dist < closest_dist)
			{
				closest_dist = dist;
				closest_seg = *it_first;
				threshold = dist;
			}
		}	
	}
	
	/*Fixing direction in case of a single segment*/
	if(closest_dist == 0 && double_way(closest_seg))
	{
		double dist_first_end_one = seg_index->distance_points(latit_first, 
			segments.at(closest_seg)->latit_begin, longit_first, 
			segments.at(closest_seg)->longit_begin);
		
		double dist_first_end_two = seg_index->distance_points(latit_first, 
			segments.at(closest_seg)->latit_end, longit_first, 
			segments.at(closest_seg)->longit_end);

		double dist_second_end_one = seg_index->distance_points(latit_second, 
			segments.at(closest_seg)->latit_begin, longit_second, 
			segments.at(closest_seg)->longit_begin);
		
		double dist_second_end_two = seg_index->distance_points(latit_second, 
			segments.at(closest_seg)->latit_end, longit_second, 
			segments.at(closest_seg)->longit_end);

		if(dist_first_end_one + dist_second_end_two > 
			dist_second_end_one + dist_first_end_two)
		{
			closest_seg = flip_segment(closest_seg);
		}
	}

	return closest_seg;
}

const double RoadNet::match_segment(const double latit, const double longit,
	const unsigned int prev_seg, const double max_distance) const
{
	std::list<unsigned int> point_neighbors;
	seg_index->k_nearest(point_neighbors, latit, longit, 
	num_neigh_match);
	
	unsigned int closest_seg = point_neighbors.back();
	double closest_dist = (n_segments + 1) * length_longest_segment;
	double dist;
	double threshold = n_segments * length_longest_segment;
	
	for(std::list<unsigned int>::iterator it = point_neighbors.begin();
		it != point_neighbors.end(); ++it)
	{
		dist = shortest_path(prev_seg, *it, threshold);
		
		if(dist < closest_dist)
		{
			closest_dist = dist;
			closest_seg = *it;
			threshold = dist;
		}
		
		if(closest_dist <= max_distance)
		{
			break;
		}

	}
	
	return closest_seg;
}

const unsigned int RoadNet::segments_within_distance(std::list<unsigned int>& ids,
	const double latit, const double longit,
	const double distance) const
{
	return seg_index->within_distance(ids, latit, longit, distance);
}


const bool PostGisIndex::connect()
{
	std::string conn_str = "dbname=" + database_name
		+ " user=" + user
		+ " password=" + password
		+ " hostaddr=" + host
		+ " port=" + port;
	
	try
	{
		conn = new pqxx::connection(conn_str.c_str());

		if(conn->is_open())
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	catch(const pqxx::broken_connection &e)
	{
		std::cerr << "Error: Cannot connect to the database " 
				<< database_name << " with user " 
				<< user << " host " << host << " port "
				<< port << std::endl << std::endl;
		std::cerr << conn_str << std::endl;
		std::cerr << e.what() << std::endl;

		return false;
	}

	return true;
}

const bool PostGisIndex::project(const double latit, const double longit,
	double& x, double& y)
{
	std::string sql;

	try
	{
		sql = "select st_x(ST_Transform(ST_GeomFromText('POINT(" +
			to_string(longit) + " " + to_string(latit) + ")', " + spatial_ref + ")," + srid + "))," + 
			"st_y(ST_Transform(ST_GeomFromText('POINT(" +
			to_string(longit) + " " + to_string(latit) + ")', " + spatial_ref + ")," + srid + "))";

		pqxx::nontransaction work(*conn);
		pqxx::result res(work.exec(sql.c_str()));

		for (pqxx::result::const_iterator r = res.begin(); r != res.end(); ++r) 
		{
			x = r[0].as<double>();
			y = r[1].as<double>();
		}
	}
	catch(const pqxx::sql_error& e)
	{
		std::cerr << "Error: Failed query:" << std::endl;
		std::cerr << sql << std::endl;
		std::cerr << e.what() << std::endl;

		return false;
	}

	return true;
}

const bool PostGisIndex::create_table()
{
	std::string sql;
	
	try
	{
		sql = "CREATE TABLE " + table_name + 
			"(id integer PRIMARY KEY, segment geometry(LINESTRING," +
			spatial_ref+"));\
			CREATE INDEX sg_idx ON " + table_name +
			 " USING GIST(segment);";
		pqxx::work work (*conn);
		work.exec(sql.c_str());
		work.commit();
	}
	catch(const pqxx::sql_error& e)
	{
		std::cerr << "Error: Failed query:" << std::endl;
		std::cerr << sql << std::endl;
		std::cerr << e.what() << std::endl;

		return false;
	}

	return true;
}

void PostGisIndex::drop_table()
{
	std::string sql = "DROP TABLE " + table_name + ";";
	
	try
	{
		sql = "DROP TABLE " + table_name + ";";
		pqxx::work work (*conn);
		work.exec(sql.c_str());
		work.commit();
	}
	catch(const pqxx::sql_error& e)
	{
		std::cerr << "Error: Failed query:" << std::endl;
		std::cerr << sql << std::endl;
		std::cerr << e.what() << std::endl;
	}
}

void PostGisIndex::disconnect()
{
	conn->disconnect();
}

/**
 * insert: inserts a segment to the database
 * @param seg segment
 * @param id segment id
**/
const bool PostGisIndex::insert(const segment* seg, const unsigned int id)
{
	//PostGis is long-lat
	std::string sql;
	
	try
	{
		sql = "INSERT INTO " + table_name + 
			"(id, segment) VALUES (" + to_string(id) +
			", ST_GeomFromText('LINESTRING(" +
			to_string(seg->longit_begin) + " " + 
			to_string(seg->latit_begin) + "," +
			to_string(seg->longit_end) + " " + 
			to_string(seg->latit_end) + ")', " + 
			spatial_ref+"));";
	
		pqxx::work work (*conn);
		work.exec(sql.c_str());
		work.commit();
	}	
	catch(const pqxx::sql_error& e)
	{
		std::cerr << "Error: Failed query:" << std::endl;
		std::cerr << sql << std::endl;
		std::cerr << e.what() << std::endl;

		return false;
	}
	
	return true;
}

const unsigned int PostGisIndex::within_distance(std::list<unsigned int>& ids,
	const double latit, const double longit,
	const double distance) const
{
	std::string sql; 
	try
	{
		sql = "SELECT id FROM " + table_name + 
			" WHERE ST_DWithin(ST_Transform(segment," + srid + "),ST_Transform(ST_GeomFromText('POINT(" + 
			to_string(longit) + " " + to_string(latit) + ")', " + spatial_ref + ")," + srid + ")," + 
			to_string(distance) + ")";
		
		pqxx::nontransaction work(*conn);
		pqxx::result res(work.exec(sql.c_str()));
		ids.clear();

		std::cout << "sql = " << sql << std::endl;

		for (pqxx::result::const_iterator r = res.begin(); r != res.end(); ++r) 
		{
			ids.push_back(r[0].as<unsigned int>());
		}
	}
	catch(const pqxx::sql_error& e)
	{
		std::cerr << "Error: Failed query:" << std::endl;
		std::cerr << sql << std::endl;
		std::cerr << e.what() << std::endl;

		return 0;
	}

	return ids.size();
}

const double PostGisIndex::distance_points(const double latit_one, 
	const double latit_two, const double longit_one, 
	const double longit_two)
{
	std::string sql; 

	return 1;

	double distance = 0;
	try
	{
		sql = "SELECT ST_Distance(ST_Transform(ST_GeomFromText('POINT(" + to_string(longit_one) + 
			" " + to_string(latit_one) + ")', " + 
			spatial_ref + ")," + srid + "), ST_Transform(ST_GeomFromText('POINT(" + 
			to_string(longit_two) + " " + to_string(latit_two) + 
			")', " + spatial_ref + ")," + srid + "));";
	
		pqxx::nontransaction work(*conn);
		pqxx::result res(work.exec(sql.c_str()));

		for (pqxx::result::const_iterator r = res.begin(); r != res.end(); ++r) 
		{
			distance = r[0].as<double>();
		}
	}
	catch(const pqxx::sql_error& e)
	{
		std::cerr << "Error: Failed query:" << std::endl;
		std::cerr << sql << std::endl;
		std::cerr << e.what() << std::endl;
	}

	return distance;
}

const unsigned int PostGisIndex::k_nearest(std::list<unsigned int>& ids, 
	const double latit, const double longit, const unsigned int k) const
{
	std::string sql;

	try
	{
		unsigned int id;
		std::string sql = "SELECT id FROM " + table_name + 
			" ORDER BY segment <-> st_setsrid(st_makepoint(" + 
			to_string(longit) + "," + to_string(latit) + 
			")," + spatial_ref + ") LIMIT " + to_string(k) + ";";
		
		pqxx::nontransaction work(*conn);
		pqxx::result res(work.exec(sql.c_str()));
		ids.clear();

		for (pqxx::result::const_iterator r = res.begin(); r != res.end(); ++r) 
		{
			r[0] >> id;
			ids.push_back(id);
		}
	}
	catch(const pqxx::sql_error& e)
	{
		std::cerr << "Error: Failed query:" << std::endl;
		std::cerr << sql << std::endl;
		std::cerr << e.what() << std::endl;

		return 0;
	}
	
	return ids.size();
}

PostGisIndex::PostGisIndex()
{
	connect();
}

const bool PostGisIndex::create()
{
	drop_table();
	
	return create_table();
}

void PostGisIndex::drop()
{
	drop_table();
}

PostGisIndex::~PostGisIndex()
{
	disconnect();
	delete conn;
}

struct compare_pair
{
	bool operator()(const std::pair<unsigned int, double>& p1, const std::pair<unsigned int, double>& p2) const
        {
		return p1.second > p2.second;
	}
};

const double RoadNet::shortest_path(const unsigned int from, const unsigned int to, 
	const double threshold) const
{
	if(from == to)
	{
		return 0;
	}

	std::vector<double> distances;
	distances.reserve(n_segments);
	boost::heap::fibonacci_heap<std::pair<unsigned int, double>, boost::heap::compare<compare_pair> > pq;
	std::vector<boost::heap::fibonacci_heap<std::pair<unsigned int, double> ,  
		boost::heap::compare<compare_pair> >::handle_type> handles;
	handles.reserve(n_segments);
	
	for(unsigned int s = 0; s < n_segments; s++)
	{
		distances.push_back(std::numeric_limits<double>::max());
	
		if(s != from)
		{
			handles.push_back(pq.push(std::pair<unsigned int, double>
				(s, std::numeric_limits<double>::max())));
		}
		else
		{
			handles.push_back(pq.push(std::pair<unsigned int, double>
				(s, 0)));
		}
	}
								
	unsigned int u;
	unsigned int z;
	double dist;
												               
	distances[from] = 0;

	while(! pq.empty())
	{
		u = pq.top().first;
		pq.pop();
		
		if(u == to)
		{
			break;
		}
		
		for (std::list<unsigned int>::iterator it = adj_list[u]->begin(); 
			it != adj_list[u]->end(); ++it)
		{
			z = *it;
			dist = distances[u] + segments.at(u)->length;
			
			if(distances[z] > dist
				&& dist <= threshold)
			{
				distances[z] = dist;

				pq.increase(handles.at(z), std::pair<unsigned int, double>(z, dist));
			}
		}
	}

	return distances[to];
}

//FIXME: optimize
const double RoadNet::shortest_path(std::list<unsigned int>& short_path, 
	const unsigned int from, const unsigned int to, 
	const double threshold) const
{
	if(from == to)
	{
		short_path.clear();
		return 0;
	}

	std::vector<double> distances;
	distances.reserve(n_segments);
	std::vector<unsigned int> reverse_edges;
	reverse_edges.reserve(n_segments);
	boost::heap::fibonacci_heap<std::pair<unsigned int, double>, boost::heap::compare<compare_pair> > pq;
	std::vector<boost::heap::fibonacci_heap<std::pair<unsigned int, double> ,  
		boost::heap::compare<compare_pair> >::handle_type> handles;
	handles.reserve(n_segments);
	
	
	for(unsigned int s = 0; s < n_segments; s++)
	{
		distances.push_back(std::numeric_limits<double>::max());
		reverse_edges.push_back(n_segments + 1);
		
		if(s != from)
		{
			handles.push_back(pq.push(std::pair<unsigned int, double>
				(s, std::numeric_limits<double>::max())));
		}
		else
		{
			handles.push_back(pq.push(std::pair<unsigned int, double>
				(s, 0)));
		}
	}
								
	unsigned int u;
	unsigned int z;
	double dist;
												               
	distances[from] = 0;
	
	while(! pq.empty())
	{
		u = pq.top().first;
		pq.pop();

		if(u == to)
		{
			break;
		}
		
		for (std::list<unsigned int>::iterator it = adj_list[u]->begin(); 
			it != adj_list[u]->end(); ++it)
		{
			z = *it;
			dist = distances[u] + segments.at(u)->length;
			
			if(distances[z] > dist
				&& dist <= threshold)
			{
				distances[z] = dist;
				pq.increase(handles.at(z), std::pair<unsigned int, double>(z, dist));
				reverse_edges[z] = u;
			}
		}
	}



	short_path.clear();

	if(distances.at(to) <= threshold)
	{
		u = reverse_edges.at(to);

		while(u != from)
		{
			short_path.push_front(u);
			u = reverse_edges.at(u);
		}
	}
	
	return distances.at(to);
}

void RoadNet::compute_segment_lengths()
{
	length_longest_segment = 0;
	
	for(unsigned int s = 0; s < segments.size(); s++)
	{
		segments.at(s)->length = seg_index->distance_points(segments.at(s)->latit_begin, 
			segments.at(s)->latit_end, segments.at(s)->longit_begin,
			segments.at(s)->longit_end);

		if(segments.at(s)->length > length_longest_segment)
		{
			length_longest_segment = segments.at(s)->length;
		}
	}
}

void RoadNet::project_segments()
{
	for(unsigned int s = 0; s < segments.size(); s++)
	{
		project(segments.at(s)->latit_begin, segments.at(s)->longit_begin,
			segments.at(s)->proj_latit_begin, segments.at(s)->proj_longit_begin);
		
		project(segments.at(s)->latit_end, segments.at(s)->longit_end,
			segments.at(s)->proj_latit_end, segments.at(s)->proj_longit_end);
	}
}


