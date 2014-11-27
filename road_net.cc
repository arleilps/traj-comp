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
#include <stdlib.h>

/*my includes*/
#include "road_net.h"
#include "io.h"

/*CONSTANTS*/

//Database
const std::string PostGisIndex::database_name = "test";
const std::string PostGisIndex::table_name = "seg_index";
const std::string PostGisIndex::host = "127.0.0.1";
const std::string PostGisIndex::port = "5432";
const std::string PostGisIndex::user = "traj_comp";
const std::string PostGisIndex::password="traj_comp";

const unsigned int RoadNet::num_short_paths = 100;

//Projection
const std::string PostGisIndex::srid = "26943";
const std::string PostGisIndex::spatial_ref = "4326";

point* new_point(const double latit, const double longit)
{
        point* p = new point;
        p->latit = latit;
        p->longit = longit;

        return p;
}

segment* new_segment
	(
		const double latit_begin, 
		const double latit_end, 
		const double longit_begin, 
		const double longit_end,
		const unsigned int p_begin, 
		const unsigned int p_end,
		const bool double_way, 
		const std::string& name
	)
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

const double std_distance(const double x1, const double x2, const double y1, const double y2)
{
	return sqrt(pow(x1-x2, 2) + pow(y1-y2,2));
}

RoadNet::RoadNet(const std::string& input_file_name)
{
	seg_id = 0;
	n_segments = 0;
	//Change the spatial index here
	seg_index = new PostGisIndex();
	read(input_file_name);
}

RoadNet::RoadNet
	(
		const std::string& input_file_name,
		const std::string& output_file_name
	)
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
			seg_ids.insert(std::pair<std::string, unsigned int>(seg->name, segments.size()));
			segments.push_back(seg);

			s++;
		}
	}
	
	//Reading adjacency list
	s = 0;
	std::list<unsigned int>* neighbors;
	distances.reserve(n_segments);

	while(s < n_segments)
	{
		std::getline(road_net_file, line_str);
		line_vec = split(line_str,',');
		neighbors = new std::list<unsigned int>;

		for(unsigned int i = 1; i < line_vec.size(); i++)
		{
			neighbors->push_back(atoi(line_vec[i].c_str()));
		}

		adj_list.push_back(neighbors);
		
		distances.push_back(new std::map<unsigned int, double>);

		s++;
	}

	//Reading pre-computed shortest path distances
	std::getline(road_net_file, line_str);
	unsigned int s1;
	unsigned int s2;
	double dist;

	while(! road_net_file.eof())
	{
		line_vec = split(line_str,',');
		
		if(line_vec.size() != 3)
		{
			std::cerr << "Error: Invalid road network file format, check the README file -- " 
				<< input_file_name << std::endl << std::endl;
			road_net_file.close();

			return false;
		}
		else
		{
			s1 = atoi(line_vec[0].c_str());
			s2 = atoi(line_vec[1].c_str());
			dist = atof(line_vec[2].c_str());

			distances.at(s1)->insert(std::pair<unsigned int, double>(s2,dist));

			std::getline(road_net_file, line_str);
		}
	}

	road_net_file.close();

	return true;
}

const bool RoadNet::create
	(
		const std::string& input_file_name,
		const std::string& output_file_name
	) 
		throw (std::ios_base::failure)
{
	try
	{
		if(build(input_file_name))
		{
			build_adjacency_list();
			index_segments();
			compute_segment_lengths();
			project_segments();
			compute_distances();
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

//segment <id,latit_begin,latit_end,longit_begin,longit_end,way,length,name>
void print_segment
	(
		segment* seg, const unsigned int id,
		std::ofstream& road_net_file
	)
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

const bool RoadNet::write(const std::string& output_file_name) 
	throw (std::ios_base::failure)
{
	std::ofstream road_net_file(output_file_name.c_str(), std::ios::out);

        if(! road_net_file.is_open())
       	{
        	std::cerr << "Error: Could not open road network file: " 
			<< output_file_name << std::endl << std::endl;
		
		return false;
        }       
	
	//Prints number of road segments
	road_net_file << n_segments << "\n";

	for(unsigned int s = 0; s < segments.size(); s++)
	{
		print_segment(segments.at(s), s, road_net_file);
	}
	
	//Prints segments adjacency list from->to
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

	//Prints shortest path distances from->to
	for(unsigned int s = 0; s < adj_list.size(); s++)
	{
		for(std::map<unsigned int, double>::iterator it = distances.at(s)->begin();
			it != distances.at(s)->end(); ++it)
		{
			road_net_file << s << "," << it->first << "," << it->second << "\n";
		}
	}
	
	return true;
}

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
			std::cerr << "Error: Invalid road network file format, check README: " 
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
			if(points[p_one]->latit != points[p_two]->latit 
				or points[p_one]->longit != points[p_two]->longit)
			{
				if(dir == "0")
				{
					//Undirected
					add_segment(id, points[p_one]->latit, 
						points[p_two]->latit,
						points[p_one]->longit, 
						points[p_two]->longit, 
						point_ids[p_one], 
						point_ids[p_two], true);
					
					id = "-1"+id; 
		
					add_segment(id, points[p_two]->latit, 
						points[p_one]->latit,
						points[p_two]->longit, 
						points[p_one]->longit,
						point_ids[p_two], 
						point_ids[p_one], true);
				}
				else
				{
					//Directed
					add_segment(id, points[p_one]->latit, 
						points[p_two]->latit,
						points[p_one]->longit, 
						points[p_two]->longit,
						point_ids[p_one], 
						point_ids[p_two], false);
				}
			}
		}

		s++;
	}
	
	n_segments = segments.size();

	for(std::map<std::string, point*>::iterator it = points.begin(); 
		it != points.end(); ++it)
	{
		delete it->second;
	}

	return true;
}

/**
 * This could be done faster using an index
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
				//Segments are connected based on points
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
		delete distances.at(s);
	}
}

void RoadNet::add_segment
	(
		const std::string id, 
		const double latit_begin, 
		const double latit_end,
		const double longit_begin, 
		const double longit_end, 
		const unsigned int p_begin,
		const unsigned int p_end, 
		const bool double_way
	)
{
	seg_ids[id] = seg_id;
	segments.push_back(new_segment(latit_begin, latit_end, longit_begin, longit_end, 
		p_begin, p_end, double_way, id));	

	seg_id++;
}

void RoadNet::index_segments()
{
	if(seg_index->create())
	{
		for(unsigned int s = 0; s < segments.size(); s++)
		{
			seg_index->insert(segments.at(s), s);
		}
	}
}

const bool RoadNet::double_way(const unsigned int seg) const
{
	return segments.at(seg)->double_way;
}

const unsigned int RoadNet::closest_segment
	(
		const double latit, 
		const double longit
	) 
		const
{
	std::list<unsigned int> points;

	seg_index->k_nearest(points, latit, longit, 1);

	return points.front();
}

const unsigned int RoadNet::segments_within_distance
	(
		std::list<unsigned int>& ids,
		const double latit, 
		const double longit,
		const double distance
	) 
		const
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

const bool PostGisIndex::project
	(
		const double latit, 
		const double longit,
		double& x, 
		double& y
	)
{
	std::string sql;

	try
	{
		sql = "select st_x(ST_Transform(ST_GeomFromText('POINT(" +
			to_string_prec(longit) + " " + to_string_prec(latit) + ")', " 
			+ spatial_ref + ")," + srid + "))," + 
			"st_y(ST_Transform(ST_GeomFromText('POINT(" +
			to_string_prec(longit) + " " + to_string_prec(latit) + 
			")', " + spatial_ref + ")," + srid + "))";

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
		//FIXME: CREATE INDEX sg_idx_"+ table_name +" ON " + table_name +
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
		std::cerr << "Error: Failed query: " << std::endl;
		std::cerr << sql << std::endl;
		std::cerr << e.what() << std::endl;
	}
}

void PostGisIndex::disconnect()
{
	conn->disconnect();
}

const bool PostGisIndex::insert
	(
		const segment* seg, 
		const unsigned int id
	)
{
	//PostGis is long-lat
	std::string sql;
	
	try
	{
		sql = "INSERT INTO " + table_name + 
			"(id, segment) VALUES (" + to_string(id) +
			", ST_GeomFromText('LINESTRING(" +
			to_string_prec(seg->longit_begin) + " " + 
			to_string_prec(seg->latit_begin) + "," +
			to_string_prec(seg->longit_end) + " " + 
			to_string_prec(seg->latit_end) + ")', " + 
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

const unsigned int PostGisIndex::within_distance
	(std::list<unsigned int>& ids,
	const double latit, 
	const double longit,
	const double distance
	) 
		const
{
	std::string sql; 
	
	try
	{
		sql = "SELECT id FROM " + table_name + 
			" WHERE ST_DWithin(ST_Transform(segment," + srid + 
			"),ST_Transform(ST_GeomFromText('POINT(" + 
			to_string_prec(longit) + " " + to_string_prec(latit) + ")', " 
			+ spatial_ref + ")," + srid + ")," + 
			to_string_prec(distance) + ")";
		
		pqxx::nontransaction work(*conn);
		pqxx::result res(work.exec(sql.c_str()));
		ids.clear();

		for (pqxx::result::const_iterator r = res.begin(); r != res.end(); ++r) 
		{
			ids.push_back(r[0].as<unsigned int>());
		}
	}
	catch(const pqxx::sql_error& e)
	{
		std::cerr << "Error: Failed query: " << std::endl;
		std::cerr << sql << std::endl;
		std::cerr << e.what() << std::endl;

		return 0;
	}

	return ids.size();
}

const double PostGisIndex::distance_points
	(
		const double latit_one, 
		const double latit_two, 
		const double longit_one, 
		const double longit_two
	)
{
	std::string sql; 

	double distance = 0;
	
	try
	{
		sql = "SELECT ST_Distance(ST_Transform(ST_GeomFromText('POINT(" + 
			to_string_prec(longit_one) + 
			" " + to_string_prec(latit_one) + ")', " + 
			spatial_ref + ")," + srid + "), ST_Transform(ST_GeomFromText('POINT(" + 
			to_string_prec(longit_two) + " " + to_string_prec(latit_two) + 
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

const double PostGisIndex::distance_point_segment
	(
		const unsigned int seg,
		const double latit, 
		const double longit
	) 
		const 
{
	std::string sql; 

	double distance = 0;
	
	try
	{
		sql = "SELECT ST_Distance(ST_GeomFromText('POINT(" + 
			to_string_prec(longit) + 
			" " + to_string_prec(latit) + ")', " + 
			srid + "), ST_Transform(segment," + srid + 
			")) FROM "+ table_name +" WHERE id = " + to_string(seg) + ";";
		/*
		sql = "SELECT ST_Distance(ST_Transform(ST_GeomFromText('POINT(" + 
			to_string_prec(longit) + 
			" " + to_string_prec(latit) + ")', " + 
			spatial_ref + ")," + srid + "), ST_Transform(segment," + srid + 
			")) FROM "+ table_name +" WHERE id = " + to_string(seg) + ";";
		*/
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

const unsigned int PostGisIndex::k_nearest
	(
		std::list<unsigned int>& ids, 
		const double latit, 
		const double longit, 
		const unsigned int k
	) 
		const
{
	std::string sql;

	try
	{
		unsigned int id;
		sql = "SELECT id FROM " + table_name + 
			" ORDER BY segment <-> st_setsrid(st_makepoint(" + 
			to_string_prec(longit) + "," + to_string_prec(latit) + 
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

const bool PostGisIndex::closest_point_segment
	(
		const unsigned int seg,
		const double latit_p, 
		const double longit_p,
		double& latit_s, 
		double& longit_s
	) 
	const
{
	std::string sql;

	try
	{
		sql = "SELECT ST_x(ST_ClosestPoint(segment, ST_GeomFromText('POINT(" +
			to_string_prec(longit_p) + " " + to_string_prec(latit_p) + ")', " +
			spatial_ref + "))),"  
			" ST_y(ST_ClosestPoint(segment, ST_GeomFromText('POINT(" +
			to_string_prec(longit_p) + " " + to_string_prec(latit_p) + ")', " +
			spatial_ref + "))) FROM " + table_name + 
			" WHERE id = " + to_string(seg) + ";";  
		
		pqxx::nontransaction work(*conn);
		pqxx::result res(work.exec(sql.c_str()));

		for (pqxx::result::const_iterator r = res.begin(); r != res.end(); ++r) 
		{
			latit_s = r[1].as<double>();
			longit_s = r[0].as<double>();
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

//Comparison function used for Fibonacci (min) heap.
struct compare_pair
{
	bool operator()(const std::pair<unsigned int, double>& p1, 
		const std::pair<unsigned int, double>& p2) const
        {
		return p1.second > p2.second;
	}
};

bool compare_pair_asc(const std::pair<unsigned int, double>* p1, 
		const std::pair<unsigned int, double>* p2)
        {
		return p1->second < p2->second;
	}

const bool RoadNet::closest_point_segment
	(
		const unsigned int seg,
		const double latit_p, 
		const double longit_p, 
		double& latit_s, 
		double& longit_s
	) 
		const
{
	seg_index->closest_point_segment
		(
			seg, 
			latit_p, 
			longit_p, 
			latit_s, 
			longit_s
		);
	
	return seg_index->project
		(
			latit_p,
			longit_p,
			latit_s,
			longit_s
		);
}

const double RoadNet::distance_point_segment
	(
		const unsigned int seg,
		const double latit, 
		const double longit
	)
		const
{
	return seg_index->distance_point_segment
		(
			seg, 
			latit, 
			longit
		);
}

void RoadNet::compute_distances()
{
	distances.reserve(segments.size());

	for(unsigned int s = 0; s < segments.size(); s++)
	{
		distances.push_back
			(
				new std::map<unsigned int, double>
			);

		shortest_path(s);
	}
}

void RoadNet::shortest_path(const unsigned int s1)
{
	std::vector<std::pair<unsigned int, double>* > distances_s1;
	//Here be dragons
	boost::heap::fibonacci_heap<std::pair<unsigned int, double>, 
		boost::heap::compare<compare_pair> > pq;
	std::vector<boost::heap::fibonacci_heap<std::pair<unsigned int, double> ,  
		boost::heap::compare<compare_pair> >::handle_type> handles;
	handles.reserve(n_segments);
	distances_s1.reserve(n_segments);
	
	for(unsigned int s = 0; s < n_segments; s++)
	{
		distances_s1.push_back
			(
				new std::pair<unsigned int, double>
					(
						s,
						std::numeric_limits<float>::max()
					)
			);
		
		if(s != s1)
		{
			handles.push_back(pq.push(std::pair<unsigned int, double>
				(s, std::numeric_limits<double>::max())));
		}
		else
		{
			handles.push_back(pq.push(std::pair<unsigned int, double>
				(s, segments.at(s1)->length)));
		}
	}

	unsigned int u;
	unsigned int z;
	double dist;
												               
	distances_s1[s1]->second = segments.at(s1)->length;

	//BFS using heap
	while(! pq.empty())
	{
		u = pq.top().first;
		pq.pop();
		
		for (std::list<unsigned int>::iterator it = adj_list[u]->begin(); 
			it != adj_list[u]->end(); ++it)
		{
			z = *it;

			dist = distances_s1[u]->second + segments.at(z)->length;

			if(distances_s1[z]->second > dist)
			{
				distances_s1[z]->second = dist;

				pq.increase(handles.at(z), 
					std::pair<unsigned int, double>(z, dist));
			}
		}
	}
	
	std::sort(distances_s1.begin(), distances_s1.end(), compare_pair_asc);
	
	for(unsigned int s = 0; s < num_short_paths; s++)
	{
		distances.at(s1)->insert(std::pair<unsigned int, double>
			(
				distances_s1[s]->first, distances_s1[s]->second)
			);
	}

	for(unsigned int s = 0; s < distances_s1.size(); s++)
	{
		delete distances_s1.at(s);
	}
}

//Dijkstra algorithm using fibonacci heap.
//The heap implementation is from boost.
//http://www.boost.org/doc/libs/1_49_0/doc/html/boost/heap/fibonacci_heap.html
const double RoadNet::shortest_path
	(
		const unsigned int s1, 
		const unsigned int s2, 
		const double proj_latit_s1,
		const double proj_latit_s2, 
		const double proj_longit_s1,
		const double proj_longit_s2,
		const double max_dist
	) 
		const
{
	if(s1 == s2)
	{
		//If segments are the same, compute distance between points
		//this might not work if the distance takes the wrong
		//way in the road.
		//return distance_points
		return std_distance
			(
				proj_latit_s1, 
				proj_latit_s2, 
				proj_longit_s1, 
				proj_longit_s2
			);
	}

	segment* from = segments.at(s1);
	segment* to = segments.at(s2);
	
	double dist_over_seg  = std_distance
		(
			proj_latit_s1,
			from->proj_latit_end,
			proj_longit_s1,
			from->proj_longit_end
		) + std_distance
		(
			proj_latit_s2,
			to->proj_latit_begin,
			proj_longit_s2,
			to->proj_longit_begin
		);

	if(distances.at(s1)->find(s2) != distances.at(s1)->end())
	{
		return distances.at(s1)->at(s2) 
			+ dist_over_seg
			- segments.at(s1)->length
			- segments.at(s2)->length;
	}

	//Here be dragons
	std::vector<double> distances_s1;
	distances_s1.reserve(n_segments);
	boost::heap::fibonacci_heap<std::pair<unsigned int, double>, 
		boost::heap::compare<compare_pair> > pq;
	std::vector<boost::heap::fibonacci_heap<std::pair<unsigned int, double> ,  
		boost::heap::compare<compare_pair> >::handle_type> handles;
	handles.reserve(n_segments);
	
	for(unsigned int s = 0; s < n_segments; s++)
	{
		distances_s1.push_back(std::numeric_limits<double>::max());
	
		if(s != s1)
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
												               
	distances_s1[s1] = 0;

	//BFS using heap
	while(! pq.empty())
	{
		u = pq.top().first;
		pq.pop();
		
		if(u == s2)
		{
			break;
		}
		
		for (std::list<unsigned int>::iterator it = adj_list[u]->begin(); 
			it != adj_list[u]->end(); ++it)
		{
			z = *it;

			dist = distances_s1[u];

			if(u != s1)
			{
				dist += segments.at(u)->length;
			}
			
			if(distances_s1[z] > dist)
			{
				distances_s1[z] = dist;

				pq.increase(handles.at(z), 
					std::pair<unsigned int, double>(z, dist));
			}
		}
	}
	
	return distances_s1[s2] + dist_over_seg;
}

//Same as the last function, but also retrieves corresponding path
const double RoadNet::shortest_path
	(
		std::list<unsigned int>& short_path, 
		const unsigned int s1, 
		const unsigned int s2, 
		const double latit_from,
		const double latit_to, 
		const double longit_from,
		const double longit_to
	) 
		const
{
	double proj_latit_s1;
	double proj_longit_s1;
	double proj_latit_s2;
	double proj_longit_s2;
	
	short_path.clear();

	std::vector<unsigned int> reverse_edges;
	reverse_edges.reserve(n_segments);

	closest_point_segment
		(
			s1, 
			latit_from, 
			longit_from, 
			proj_latit_s1, 
			proj_longit_s1
		);
	
	closest_point_segment
		(
			s2, 
			latit_to, 
			longit_to, 
			proj_latit_s2, 
			proj_longit_s2
		);
	
	segment* from = segments.at(s1);
	segment* to = segments.at(s2);
	
	double dist_over_seg  = std_distance
		(
			proj_latit_s1,
			from->proj_latit_end,
			proj_longit_s1,
			from->proj_longit_end
		) + std_distance
		(
			proj_latit_s2,
			to->proj_latit_begin,
			proj_longit_s2,
			to->proj_longit_begin
		);
	
	if(s1 == s2)
	{
		return std_distance
			(
				proj_latit_s1, 
				proj_latit_s2, 
				proj_longit_s1, 
				proj_longit_s2
			);
	}

	std::vector<double> distances_s1;
	distances_s1.reserve(n_segments);
	boost::heap::fibonacci_heap<std::pair<unsigned int, double>, 
		boost::heap::compare<compare_pair> > pq;
	std::vector<boost::heap::fibonacci_heap<std::pair<unsigned int, double> ,  
		boost::heap::compare<compare_pair> >::handle_type> handles;
	handles.reserve(n_segments);
	
	for(unsigned int s = 0; s < n_segments; s++)
	{
		distances_s1.push_back(std::numeric_limits<double>::max());
		reverse_edges.push_back(0);

		if(s != s1)
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
												               
	distances_s1[s1] = 0;
	
	while(! pq.empty())
	{
		u = pq.top().first;
		pq.pop();
		
		if(u == s2)
		{
			break;
		}
		
		for (std::list<unsigned int>::iterator it = adj_list[u]->begin(); 
			it != adj_list[u]->end(); ++it)
		{
			z = *it;

			dist = distances_s1[u];

			if(u != s1)
			{
				dist += segments.at(u)->length; 
			}
			
			
			if(distances_s1[z] > dist)
			{
				distances_s1[z] = dist;

				pq.increase(handles.at(z), 
					std::pair<unsigned int, double>(z, dist));
				reverse_edges[z] = u;
			}
		}
	}
	
	if(distances_s1[s2] < std::numeric_limits<double>::max())
	{
		u = reverse_edges.at(s2);

		while(u != s1)
		{
			short_path.push_front(u);
			u = reverse_edges.at(u);
		}
	}

	return distances_s1[s2] + dist_over_seg;
}

void RoadNet::compute_segment_lengths()
{
	length_longest_segment = 0;
	
	for(unsigned int s = 0; s < segments.size(); s++)
	{
		segments.at(s)->length = seg_index->distance_points
			(
				segments.at(s)->latit_begin, 
				segments.at(s)->latit_end, 
				segments.at(s)->longit_begin,
				segments.at(s)->longit_end
			);

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
		project
			(
				segments.at(s)->latit_begin, 
				segments.at(s)->longit_begin,
				segments.at(s)->proj_latit_begin, 
				segments.at(s)->proj_longit_begin
			);
		
		project
			(
				segments.at(s)->latit_end, 
				segments.at(s)->longit_end,
				segments.at(s)->proj_latit_end, 
				segments.at(s)->proj_longit_end
			);
	}
}

