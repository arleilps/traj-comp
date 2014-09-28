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
 *	FILE: database.cc: Database interface
**/

/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <iostream>
#include <pqxx/pqxx>

#include "database.h"

const std::string DataSegPostgresql::database_name = "test";
const std::string DataSegPostgresql::table_name = "seg_traj";
const std::string DataSegPostgresql::host = "127.0.0.1";
const std::string DataSegPostgresql::port = "5432";
const std::string DataSegPostgresql::user = "traj_comp";
const std::string DataSegPostgresql::password="traj_comp";
const std::string DataSegPostgresql::spatial_ref = "4326";

const std::string DataPointsPostgresql::database_name = "test";
const std::string DataPointsPostgresql::table_name = "points_traj";
const std::string DataPointsPostgresql::host = "127.0.0.1";
const std::string DataPointsPostgresql::port = "5432";
const std::string DataPointsPostgresql::user = "traj_comp";
const std::string DataPointsPostgresql::password="traj_comp";
const std::string DataPointsPostgresql::spatial_ref = "4326";

query_result* new_query_result(const std::string object, const std::string time)
{
	query_result* res = new query_result;

	res->object = object;
	res->time = time;

	return res;
}

DataPointsPostgresql::DataPointsPostgresql()
{
	if(connect())
	{
		create_table();
	}
}

DataPointsPostgresql::~DataPointsPostgresql()
{
	 //FIXME
	 drop_table();
	 disconnect();
	 delete conn;
}

const bool DataPointsPostgresql::insert(const std::string object,
	const std::string latit,
	const std::string longit,
	const std::string time)
{
	std::string sql;
	try
	{
		sql = "INSERT INTO " + table_name +
			"(id, time, point, speed) VALUES ('" + object +
			"', TO_TIMESTAMP("+ time +")\
			, ST_GeomFromText('POINT(" +
			longit + " " +
			latit + ")', " +
			spatial_ref + "), 0);";
	
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

const unsigned int DataPointsPostgresql::select(std::list<query_result*>& results,
	const std::string latit,
	const std::string longit,
	const std::string distance,
        const std::string time_begin,
        const std::string time_end)
{
	std::string sql;
	try
	{
		if(time_begin != "" and time_end != "")
		{
			sql = "SELECT id, time FROM " + table_name +
				" WHERE ST_DWithin(point, ST_GeomFromText('POINT(" +
				longit + " " + latit + ")', " + spatial_ref + ")," +
				distance + 
				") AND time >= TO_TIMESTAMP("+time_begin+") AND time <= TO_TIMESTAMP("+time_end+");";
		}
		else
		{
			sql = "SELECT id, time FROM " + table_name +
				" WHERE ST_DWithin(point, ST_GeomFromText('POINT(" +
				longit + " " + latit + ")', " + spatial_ref + ")," +
				distance + ");";
			
			std::cout << "query: " << sql << std::endl;
		}

		pqxx::nontransaction work(*conn);
		pqxx::result res(work.exec(sql.c_str()));
		results.clear();
		query_result* result;
	
		for (pqxx::result::const_iterator r = res.begin(); r != res.end(); ++r)
		{
			result = new_query_result(r[0].as<std::string>(), r[1].as<std::string>());
			results.push_back(result);
		}
	}
	catch(const pqxx::sql_error& e)
	{
		std::cerr << "Error: Failed query:" << std::endl;
		std::cerr << sql << std::endl;
		std::cerr << e.what() << std::endl;

		return 0;
	}

	return results.size();
}

const bool DataPointsPostgresql::connect()
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

void DataPointsPostgresql::disconnect()
{
	conn->disconnect();
}

const bool DataPointsPostgresql::create_table()
{
	std::string sql;

	try
	{
		sql = "CREATE TABLE " + table_name +
			"(id varchar(60) PRIMARY KEY, \
			time timestamp, \
			point geometry(POINT, " + spatial_ref +
			"), speed real);\
			CREATE INDEX pt_idx ON " + table_name +
			" USING GIST(point);\
			CREATE INDEX t_idx ON " + table_name +
			"(time);";
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

void DataPointsPostgresql::drop_table()
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

DataSegPostgresql::DataSegPostgresql()
{
	if(connect())
	{
		create_table();
	}
}

DataSegPostgresql::~DataSegPostgresql()
{
	//FIXME
	drop_table();
	disconnect();
	delete conn;
}

const bool DataSegPostgresql::insert(const std::string object,
		const std::string segment,
		const std::string time)
{
	std::string sql;
	try
	{
		sql = "INSERT INTO " + table_name +
			"(id, segment, time, speed) VALUES ('" + object +
			"','" + segment + "', TO_TIMESTAMP("+time+"), 0);";

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

const unsigned int DataSegPostgresql::select(std::list<query_result*>& results,
		const std::string segment,
		const std::string time_begin,
		const std::string time_end)
{
	std::string sql;
	try
	{
		if(time_begin != "" and time_end != "")
		{
			sql = "SELECT id, time FROM " + table_name +
				" WHERE segment = '" + segment +
				"' AND time >= TO_TIMESTAMP("+time_begin+") AND time <= TO_TIMESTAMP("+time_end+");";
		}
		else
		{
			sql = "SELECT id, time FROM " + table_name +
				" WHERE segment = '" + segment + "';";
		}

		pqxx::nontransaction work(*conn);
		pqxx::result res(work.exec(sql.c_str()));
		results.clear();
		query_result* result;

		for (pqxx::result::const_iterator r = res.begin(); r != res.end(); ++r)
		{
			result = new_query_result(r[0].as<std::string>(), r[1].as<std::string>());
			results.push_back(result);
		}
	}
	catch(const pqxx::sql_error& e)
	{
		std::cerr << "Error: Failed query:" << std::endl;
		std::cerr << sql << std::endl;
		std::cerr << e.what() << std::endl;

		return 0;
	}

	return results.size();
}

const bool DataSegPostgresql::connect()
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

void DataSegPostgresql::disconnect()
{
	conn->disconnect();
}

const bool DataSegPostgresql::create_table()
{
	std::string sql;

	try
	{
		sql = "CREATE TABLE " + table_name +
			"(id varchar(60) PRIMARY KEY, \
			time timestamp(5), \
			segment varchar(60),\
			speed real);\
			CREATE INDEX sg2_idx ON " + table_name +
			"(segment);\
			CREATE INDEX t_idx ON " + table_name +
			"(time);";
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

void DataSegPostgresql::drop_table()
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

