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
std::string Parameters::conf_file_name;
std::string Parameters::shortest_path_file_name;
std::string Parameters::query_file_name;
std::string Parameters::trajectory_folder;
unsigned int Parameters::max_length_subt;
unsigned int Parameters::order;
unsigned int Parameters::min_sup;
unsigned int Parameters::max_shortest_path;
unsigned int Parameters::delay;
unsigned int Parameters::num_threads;
double Parameters::error;
double Parameters::lambda;
unsigned int Parameters::num_iterations;

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

const std::vector<std::string> split(const std::string &s)
{
        std::stringstream ss(s);
        std::string item;
        std::vector<std::string> elems;
      
        while(std::getline(ss, item, ' '))
        {
                elems.push_back(item);
        }
           
        return elems;
}

void print_statistics(TrajDB* traj_db)
{
	std::cout << "inserts_second = " << traj_db->inserts_second() << std::endl;
	std::cout << "queries_second = " << traj_db->queries_second() << std::endl;
	std::cout << "insert_time = " << traj_db->insert_time() << std::endl;
	std::cout << "training_time = " << traj_db->training_time() << std::endl;
	std::cout << "query_time = " << traj_db->query_time() << std::endl;
	std::cout << "num_updates_orig = " << traj_db->num_updates_orig() << std::endl;
	std::cout << "num_updates_inserted = " << traj_db->num_updates_inserted() << std::endl;
	std::cout << "num_updates_train = " << traj_db->num_updates_train() << std::endl;
	std::cout << "num_queries = " << traj_db->num_queries() << std::endl;
	std::cout << "num_traj_inserted = " << traj_db->num_traj_inserted() << std::endl;
	std::cout << "num_traj_train = " << traj_db->num_traj_train() << std::endl;
	std::cout << "compression_ratio = " << traj_db->compression_ratio() << std::endl;
}

void print_statistics(TrajCompAlgo* algo)
{
	std::cout << "compression_time = " << algo->compression_time() << std::endl;
	std::cout << "training_time = " << algo->training_time() << std::endl;
	std::cout << "num_updates_orig = " << algo->num_updates_orig() << std::endl;
	std::cout << "num_updates_comp = " << algo->num_updates_comp() << std::endl;
	std::cout << "num_updates_train = " << algo->num_updates_train() << std::endl;
	std::cout << "num_traj_comp = " << algo->num_traj_comp() << std::endl;
	std::cout << "num_traj_train = " << algo->num_traj_train() << std::endl;
	std::cout << "compression_ratio = " << algo->compression_ratio() << std::endl;
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
	std::cout << " -q, --query		output file" << std::endl;
	std::cout << " -c, --compression	compression algorithm [MAP,FS,SP,SPFS,PPM]" << std::endl;
	std::cout << " -u, --length-subt	max length subtrajectory [in segments]" << std::endl;
	std::cout << " -r, --order		order" << std::endl;
	std::cout << " -s, --min-sup		minimum support subtrajectory [integer]" << std::endl;
	std::cout << " -p, --short-path		max length shortest path [in meters]" << std::endl;
	std::cout << " -f, --potgis-conf	postgis configuration file" << std::endl;
	std::cout << " -d, --gps-updates	gps updates file to be map-matched" << std::endl;
	std::cout << " -m, --error		temporal compression error" << std::endl;
	std::cout << " -l, --short-file		short paths file" << std::endl;
	std::cout << " -a, --delay		delay for late update compression" << std::endl;
	std::cout << " -n, --num-threads	number of threads" << std::endl;
	std::cout << " -i, --num-iter		number of iterations EM" << std::endl;
	std::cout << " -f, --postgis-conf	postgis configuration file" << std::endl;
	std::cout << " -j, --traj-folder	folder to output trajectories" << std::endl;
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
	min_sup = 1;
	max_shortest_path = 0;
	num_threads = 1;
	lambda = 0;
	error = 0;
	delay =  std::numeric_limits<unsigned int>::max();
	num_iterations = 1;

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
		>> GetOpt::Option('q', "query", query_file_name, "")
		>> GetOpt::Option('c', "compression", compression_algorithm, "")
		>> GetOpt::Option('d', "gps-updates", gps_file_name, "")
		>> GetOpt::Option('f', "postgis-conf", conf_file_name, "../data/sfo_postgis.conf")
		>> GetOpt::Option('l', "short-file", shortest_path_file_name, "")
		>> GetOpt::Option('u', "length-subt", max_length_subt)
		>> GetOpt::Option('r', "order", order)
		>> GetOpt::Option('s', "min-sup", min_sup)
		>> GetOpt::Option('p', "short-path", max_shortest_path)
		>> GetOpt::Option('m', "error", error)
		>> GetOpt::Option('a', "delay", delay)
		>> GetOpt::Option('n', "num-threads", num_threads)
		>> GetOpt::Option('i', "num_iter", num_iterations)
		>> GetOpt::Option('j', "traj-folder", trajectory_folder, "")
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
	std::cout << "road_network = " << road_net_file_name << std::endl;
	std::cout << "training_trajectories = " << training_traj_file_name << "\n";
	std::cout << "test_trajectories = " << test_traj_file_name << "\n";
	std::cout << "query_file_name = " << test_traj_file_name << "\n";
	std::cout << "compression_algorithm = " << compression_algorithm << std::endl;
	std::cout << "output = " << output_file_name << std::endl;
	std::cout << "length_subtrajectories = " << max_length_subt << std::endl;
	std::cout << "order = " << order << std::endl;
	std::cout << "min_support = " << min_sup << std::endl;
	std::cout << "shortest_paths = " << max_shortest_path << std::endl;
	std::cout << "gps_updates_file = " << gps_file_name << std::endl;
	std::cout << "conf_file = " << conf_file_name << std::endl;
	std::cout << "delay = " << delay << std::endl;
	std::cout << "number_of_threads = " << num_threads << std::endl;
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

const double string_to_double(const std::string& s)
{
	double d;
	std::stringstream ss(s);
	ss >> d;

	return d;
}
