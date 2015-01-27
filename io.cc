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
 *	FILE io.cc: Implementation of I/O operations.
**/

/*my includes*/
#include "io.h"

/*Setting default values for the parameters*/
std::string Parameters::road_net_file_name;
std::string Parameters::training_traj_file_name;
std::string Parameters::test_traj_file_name;
std::string Parameters::output_file_name;
std::string Parameters::compression_algorithm;
std::string Parameters::gps_file_name;
unsigned int Parameters::max_length_subt;
unsigned int Parameters::order;
unsigned int Parameters::min_sup;
unsigned int Parameters::max_shortest_path;
unsigned int Parameters::num_threads;

std::vector<std::string> Parameters::compression_algorithms;

/**
 * Splits a string using a delimiter
 * @param s string
 * @param delim delimiter
 * @return vector with substrings
 * @throws 
**/
const std::vector<std::string> split(const std::string &s, char delim)
{
        std::stringstream ss(s);
        std::string item;
        std::vector<std::string> elems;
      
        while(std::getline(ss, item, delim))
        {
                elems.push_back(item);
        }
           
        return elems;
}

/**
 * Prints the usage of this program
 * @param
 * @return
 * @throws
**/
void Parameters::print_usage()
{
	std::cout << "Usage: ./traj_comp [OPTIONS]..." << std::endl;
	std::cout << "Learning how to compress real-time trajectory data" << std::endl;
	std::cout << " -g, --net		road network file" << std::endl;
	std::cout << " -t, --train		training trajectories file" << std::endl;
	std::cout << " -e, --test		test trajectories file" << std::endl;
	std::cout << " -o, --output		output file" << std::endl;
	std::cout << " -c, --compression	compression algorithm" << std::endl;
	std::cout << " -u, --length-subt	max length subtrajectory" << std::endl;
	std::cout << " -r, --order		order" << std::endl;
	std::cout << " -s, --min-sup		minimum support subtrajectory" << std::endl;
	std::cout << " -p, --short-path		max length shortest path" << std::endl;
	std::cout << " -d, --gps-updates	gps updates file to be map-matched" << std::endl;
	std::cout << " -n, --num-threads	number of threads" << std::endl;
	std::cout << " -h, --help		shows this help" << std::endl;
}

/**
 * Reads the input parameters for the compression
 * @param argc size of the command line
 * @param argv string command line
 * @return 
 * @throws invalidParameterSettingException
**/
bool Parameters::read(int argc, char** argv) throw (InvalidParameterSettingException)
{
	InvalidParameterSettingException invalid_parameters;
	max_length_subt = 0;
	order = 0;
	min_sup = std::numeric_limits<unsigned int>::max();
	max_shortest_path = 0;

	try
	{
		GetOpt::GetOpt_pp ops(argc, argv);
 
		if (ops >> GetOpt::OptionPresent('h', "help"))
		{
			print_usage();
			
			return false;
		}

		ops >> GetOpt::Option('g', "net", road_net_file_name, "")
		>> GetOpt::Option('t', "train", training_traj_file_name, "")
		>> GetOpt::Option('e', "test", test_traj_file_name, "")
		>> GetOpt::Option('o', "output", output_file_name, "")
		>> GetOpt::Option('c', "compression", compression_algorithm, "")
		>> GetOpt::Option('d', "gps-updates", gps_file_name, "")
		>> GetOpt::Option('u', "length-subt", max_length_subt)
		>> GetOpt::Option('r', "order", order)
		>> GetOpt::Option('s', "min-sup", min_sup)
		>> GetOpt::Option('p', "short-path", max_shortest_path)
		>> GetOpt::Option('n', "num-threads", num_threads)
		;
	}
	catch(GetOpt::GetOptEx& e)
	{
		std::cerr << "Fatal error while parsing the command line parameters!" << std::endl;
		std::cerr << "Try \'./traj_comp --help\' for more information." << std::endl;
		throw invalid_parameters;
	}

	return true;
}

/**
 * Prints the input parameters on the terminal
 * @param
 * @return
 * @throws
**/
void Parameters::print()
{
	std::cout << "road network: " << road_net_file_name << std::endl;
	std::cout << "training trajectories: " << training_traj_file_name << "\n";
	std::cout << "test trajectories: " << test_traj_file_name << "\n";
	std::cout << "compression algorithm: " << compression_algorithm << std::endl;
	std::cout << "output: " << output_file_name << std::endl;
	std::cout << "length subtrajectories: " << max_length_subt << std::endl;
	std::cout << "order: " << order << std::endl;
	std::cout << "min support: " << min_sup << std::endl;
	std::cout << "shortest-paths: " << max_shortest_path << std::endl;
	std::cout << "gps updates file: " << gps_file_name << std::endl;
	std::cout << "number of threads: " << num_threads << std::endl;
}

/**
 * Sets the list of valid compression algorithms, so that 
 * an algorithm given as input can be checked
 * @param algorithms vector with the valid algorithm identifiers
 * @return
 * @throws
 **/
void Parameters::set_compression_algorithms(std::vector<std::string>& algorithms)
{
	compression_algorithms = algorithms;
}

const std::string to_string_prec(const double t)
{
	std::ostringstream stm;
	stm << std::setprecision( PREC );
	stm << t;

	return stm.str() ;
}

