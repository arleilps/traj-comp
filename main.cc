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
	/*Setting the compression algorithms*/
	std::vector<std::string> compression_algorithms;
	compression_algorithms.push_back("IND");  	//Index road network
	compression_algorithms.push_back("INDSP");  	//Index road shortest paths
	compression_algorithms.push_back("MAP");  	//Map-match trajectories
	compression_algorithms.push_back("SPFS");	// Shortest path 
							//+ frequent subtrajectories
	compression_algorithms.push_back("PPM");	//Prediction by partial matching
	compression_algorithms.push_back("NSTD");	//Network synchronized time distance
	compression_algorithms.push_back("EM");		//EM compression
	compression_algorithms.push_back("ONTRAC-FULL");	//ONTRAC with full decompression
	compression_algorithms.push_back("ONTRAC-PART");	//ONTRAC with partial decompression
	compression_algorithms.push_back("NONE");	//No compression

	Parameters::set_compression_algorithms(compression_algorithms);
	unsigned int num_updates;	
	TrajCompAlgo* alg;
	TrajDB* traj_db;

	/*Reading the input parameters*/
	if(Parameters::read(argc,argv))
	{
		Trajectory::set_num_threads(Parameters::num_threads);
		PostGisIndex::set_config(Parameters::conf_file_name);
		TrajDBPostGis::set_config(Parameters::conf_file_name);
		RoadNet* net;

		if(Parameters::compression_algorithm == "IND"
			or Parameters::compression_algorithm == "INDSP")
		{
			if(Parameters::compression_algorithm == "IND")
			{
				//Indexing road network
				net = new RoadNet(Parameters::road_net_file_name,
					Parameters::output_file_name);
			}
			else
			{
				net = new RoadNet(Parameters::road_net_file_name);
				
				net->write_short_path_struct
					(
						Parameters::shortest_path_file_name,
						Parameters::max_shortest_path, 
						Parameters::num_threads
					);
			}
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
				if(Parameters::compression_algorithm == "SPFS")
				{
					alg = new ShortestPathFreqSubt(
						Parameters::min_sup, Parameters::max_length_subt, net,
						Parameters::shortest_path_file_name, Parameters::delay);
				}

				if(Parameters::compression_algorithm == "PPM")
				{
					alg = new PredPartMatch(Parameters::order, net);
				}

				if(Parameters::compression_algorithm == "NSTD")
				{
					alg = new NSTD(Parameters::error, net);
				}

				if(Parameters::compression_algorithm == "EM")
				{
					alg = new EM(Parameters::error, net, 
							Parameters::num_iterations, 5.0,
							Parameters::output_file_name,
							Parameters::num_threads
						);
				}

				if(Parameters::compression_algorithm == "ONTRAC-FULL")
				{
					traj_db = new OntracFull(Parameters::order,
							Parameters::error, 
							net, Parameters::num_iterations, 5.0,
							Parameters::output_file_name,
							Parameters::num_threads
						);
					
					traj_db->create();
					traj_db->train(Parameters::training_traj_file_name);
					traj_db->insert(Parameters::test_traj_file_name);
					traj_db->where_at(Parameters::query_file_name,
						Parameters::output_file_name);
					print_statistics(traj_db);
					traj_db->drop();
					delete traj_db;
				}
				
				if(Parameters::compression_algorithm == "ONTRAC-PART")
				{
					traj_db = new OntracPart(Parameters::order, Parameters::error, 
							net, Parameters::num_iterations, 5.0,
							Parameters::output_file_name,
							Parameters::num_threads
						);
					
					traj_db->create();
					traj_db->train(Parameters::training_traj_file_name);
					traj_db->insert(Parameters::test_traj_file_name);
					traj_db->where_at(Parameters::query_file_name, 
						Parameters::output_file_name);
					print_statistics(traj_db);
					traj_db->drop();
					delete traj_db;
				}
				
				if(Parameters::compression_algorithm == "NONE")
				{
					traj_db = new TrajDB(net);
					traj_db->create();
					traj_db->insert(Parameters::test_traj_file_name);
					traj_db->where_at(Parameters::query_file_name,
						Parameters::output_file_name);
					print_statistics(traj_db);
					traj_db->drop();
					delete traj_db;
				}

				if(Parameters::compression_algorithm != "ONTRAC-FULL"
					&& Parameters::compression_algorithm != "ONTRAC-PART"
					&& Parameters::compression_algorithm != "NONE"
					)
				{
					alg->test(Parameters::test_traj_file_name);
					print_statistics(alg);
					delete alg;
				}

			}
		}

		Parameters::print();

		delete net;
	}

	return 0;
}

