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
 *	FILE main.cc: Main method.
**/

/*std includes*/
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

/*my includes*/
#include "io.h"
#include "road_net.h"
#include "traj_comp.h"

#include "test_road_net.h"
#include "test_moving_obj.h"
#include "test_traj_comp.h"

/**
 * Main method
 * @param argc size of the command line
 * @param argv string containing the command line
 * @return 0 if everything works, 1 otherwise
 * @throws
**/

int main(int argc, char** argv)
{
	PostGisIndex::set_config("../data/sfo_postgis.conf");
//	test_road_net();
//	test_database();
//	test_moving_obj();
//	test_moving_obj_file();
//	test_traj_comp_freq_subt_sf_cab();
//	test_traj_comp_freq_subt();
//	test_traj_comp_ppm();
//	test_traj_comp_short_path();
//	test_traj_comp_short_path_freq_subt();
//	test_dist_times();
//	test_moving_obj_traj_file();
//	test_TSND();
//	test_least_squares();

//	exit(1);

	/*Setting the compression algorithms*/
	std::vector<std::string> compression_algorithms;
	compression_algorithms.push_back("IND");  	//Index road network
	compression_algorithms.push_back("MAP");  	//Map-match trajectories
	compression_algorithms.push_back("FS");	  	//Frequent subtrajectories
	compression_algorithms.push_back("SP");	  	//Shortest path
	compression_algorithms.push_back("SPFS");	// Shortest path 
							//+ frequent subtrajectories
	compression_algorithms.push_back("PPM");	//Prediction by partial matching
	compression_algorithms.push_back("NSTD");	//Network synchronized time distance
	compression_algorithms.push_back("LS");		//Least-squares
	
	Parameters::set_compression_algorithms(compression_algorithms);
	unsigned int num_updates;	
	TrajCompAlgo* alg;

	/*Reading the input parameters*/
	if(Parameters::read(argc,argv))
	{
		Trajectory::set_num_threads(Parameters::num_threads);
		PostGisIndex::set_config(Parameters::conf_file_name);
		RoadNet* net;

		if(Parameters::compression_algorithm == "IND")
		{
			//Indexing road network
			net = new RoadNet(Parameters::road_net_file_name,
				Parameters::output_file_name);
		}
		else
		{
			//Reading already indexed road network
			net = new RoadNet(Parameters::road_net_file_name);

			if(Parameters::compression_algorithm == "MAP")
			{
				Trajectory::write_map_matched_trajectories_multithreads(
					Parameters::gps_file_name, 
					Parameters::output_file_name, net);
			}
			else
			{
				if(Parameters::compression_algorithm == "FS")
				{
					alg = new FreqSubt(Parameters::min_sup, 
						Parameters::max_length_subt, net, 
						Parameters::delay);
				}

				if(Parameters::compression_algorithm == "SP")
				{
					alg = new ShortestPath(Parameters::max_shortest_path, net,
						Parameters::num_threads, Parameters::delay);
				}

				if(Parameters::compression_algorithm == "SPFS")
				{
					alg = new ShortestPathFreqSubt(Parameters::max_shortest_path, 
						Parameters::min_sup, Parameters::max_length_subt, net,
						Parameters::num_threads, Parameters::delay);
				}

				if(Parameters::compression_algorithm == "PPM")
				{
					alg = new PredPartMatch(Parameters::order, net);
				}

				if(Parameters::compression_algorithm == "NSTD")
				{
					alg = new NSTD(Parameters::error, net);
				}

				if(Parameters::compression_algorithm == "LS")
				{
					alg = new LeastSquares(Parameters::error, 
						Parameters::lambda, net);
				}

				alg->train(Parameters::training_traj_file_name);

				alg->test(Parameters::test_traj_file_name);

				print_statistics(alg);

				delete alg;
			}
		}

		Parameters::print();

		delete net;
	}

	return 0;
}

