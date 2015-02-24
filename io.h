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
 *	FILE io.h: Definitions of classes related to I/O.
**/

#ifndef IO_H
#define IO_H

/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <algorithm>
#include <iostream>
#include <limits.h>
#include <iomanip>
#include <limits>

/*my includes*/
#include "getopt_pp.h"
#include "traj_comp.h"

#define PREC 10

/**
 * Parameter setting exception
**/
class InvalidParameterSettingException: public std::exception
{
	virtual const char* what() const throw()
	{
		return "Invalid input parameter setting!";
	}
};

/**
 * Splits a string using a delimiter
 * @param s string
 * @param delim delimiter
 * @return vector with substrings
 * @throws 
**/
const std::vector<std::string> split(const std::string &s, char delim);

const std::vector<std::string> split(const std::string &s);

template < typename T > std::string to_string( const T& t)
{
	std::ostringstream stm;
	stm << t;
	
	return stm.str() ;
}

const std::string to_string_prec(const double t);

const double string_to_double(const std::string& s);

void print_statistics(TrajCompAlgo* algo);

/**
 * Simple handler for the input parameters
**/
class Parameters
{
	public:
		/*Input parameters*/
		static std::string road_net_file_name;
		static std::string training_traj_file_name;
		static std::string test_traj_file_name;
		static std::string output_file_name;
		static std::string compression_algorithm;
		static std::string gps_file_name;
		static std::string conf_file_name;
		static unsigned int max_length_subt;
		static unsigned int order;
		static unsigned int min_sup;
		static unsigned int max_shortest_path;
		static unsigned int num_threads;
		
		/*List of valid compression algorithms for checking*/
		static std::vector < std::string > compression_algorithms;
		
		/**
		 * Reads the input parameters for the compression
		 * @param argc size of the command line
		 * @param argv string command line
		 * @return 
		 * @throws invalidParameterSettingException
		**/
		static bool read(int argc, char** argv) throw (InvalidParameterSettingException);
		
		/**
		 * Prints the input parameters on the terminal
		 * @param
		 * @return
		 * @throws
		**/
		static void print();

		/**
		 * Sets the list of valid compression algorithms, so that 
		 * an algorithm given as input can be checked
		 * @param algorithms vector with the valid algorithm identifiers
		 * @return
		 * @throws
		 **/
		static void set_compression_algorithms(std::vector<std::string>& algorithms);

		/**
		 * Prints the usage of this program
		 * @param
		 * @return
		 * @throws
		**/
		static void print_usage();
};

#endif
