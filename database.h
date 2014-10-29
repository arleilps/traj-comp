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
 *	FILE: database.h: Database interface
**/

#ifndef DATABASE_H
#define DATABASE_H

/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <list>
#include <map>
#include <algorithm>
#include <iostream>
#include <pqxx/pqxx>

typedef struct t_query_result
{
	std::string object;
	std::string time;
}query_result;

query_result* new_query_result(const std::string object, const std::string time);

/**
 * Database for segments
**/
class DataSeg
{
	public:
		DataSeg(){};
		
		virtual ~DataSeg(){};
		
		virtual const bool create()
		{
			return false;
		}

		virtual const bool drop()
		{
			return false;
		}
		
		virtual const bool insert(const std::string object, 
			const std::string segment, 
			const std::string time)
		{
			return false;
		}

		virtual const unsigned int select(std::list<query_result*>& results,
			const std::string segment,
			const std::string time_begin="",
			const std::string time_end="")
		{
			return false;
		}
};

/**
 * Postgresql database for segments
**/
class DataSegPostgresql: public DataSeg
{
	public:	
		DataSegPostgresql();
		
		virtual ~DataSegPostgresql();
		
		const bool create();
		
		const bool drop();

		const bool insert(const std::string object, 
			const std::string segment, 
			const std::string time);

		const unsigned int select(std::list<query_result*>& results,
			const std::string segment,
			const std::string time_begin="",
			const std::string time_end="");
	private:
		pqxx::connection* conn;
		static const std::string database_name;
		static const std::string table_name;
		static const std::string host;
		static const std::string port;
		static const std::string user;
		static const std::string password;
		static const std::string spatial_ref;

		const bool connect();
		void disconnect();
};


#endif
