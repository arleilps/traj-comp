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
 *	FILE test_database.cc: Test database interface
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

/*my includes*/
#include "database.h"
#include "test_database.h"
#include "io.h"

const bool test_database()
{
	std::cout << "Testing the database interface" << std::endl;
	std::string object;
	std::string segment;
	unsigned int time;
	
	/*Creating a point database*/
	std::cout << "Creating a segment database" << std::endl;
	DataSeg* dseg = new DataSegPostgresql();
	dseg->create();
	std::cout << "Segment database created" << std::endl;
	
	/*Inserting segments*/
	std::cout << "Insert" << std::endl;
	
	//aldhidd seg1 1211018404
	object = "aldhidd";
	segment = "seg1";
	time = 1211018404;

	dseg->insert(object, segment, to_string(time));

	//aydwaho 37.74798 -122.4157 1211018404
	object = "aydwaho";
	segment = "seg2";
	time = 1211018405;
	
	dseg->insert(object, segment, to_string(time));

	//eapceou seg3 1211018404
	object = "eapceou";
	segment = "seg1";
	time = 1211018406;

	dseg->insert(object, segment, to_string(time));
	
	std::cout << "Select" << std::endl;
	
	unsigned int count;
	std::list<query_result*> results;
	unsigned int time_begin;
	unsigned int time_end;

	segment = "seg1";

	count = dseg->select(results, segment);

	std::cout << count << " objects at " << segment << std::endl; 

	for(std::list<query_result*>::iterator it = results.begin();
		it != results.end(); ++it)
	{
		std::cout << "object: " << (*it)->object << " time: " << (*it)->time << std::endl;
		delete (*it);
	}

	segment = "seg2";

	count = dseg->select(results, segment);

	std::cout << count << " objects at " << segment << std::endl; 

	for(std::list<query_result*>::iterator it = results.begin();
		it != results.end(); ++it)
	{
		std::cout << "object: " << (*it)->object << " time: " << (*it)->time << std::endl;
		delete (*it);
	}

	segment = "seg1";
	time_begin = 1211018404;
	time_end = 1211018405;
	
	count = dseg->select(results, segment, to_string(time_begin), to_string(time_end));
	
	std::cout << count << " objects at " << segment << " between time " << time_begin << 
		" and " << time_end << std::endl;

	for(std::list<query_result*>::iterator it = results.begin();
		it != results.end(); ++it)
	{
		std::cout << "object: " << (*it)->object << " time: " << (*it)->time << std::endl;
		delete (*it);
	}
		
	std::cout << "Deleting segment database" << std::endl;
	dseg->drop();
	delete dseg;
	std::cout << "Segment database deleted" << std::endl;
	
	std::cout << "Test finished" << std::endl;

	return true;
}

