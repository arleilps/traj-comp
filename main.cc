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
//	test_road_net();
//	test_database();
//	test_moving_obj();
//	test_moving_obj_traj_file();
	test_traj_comp_freq_subt_sf_cab();

	/*Setting the compression algorithms*/
//	std::vector<std::string> compression_algorithms;
//	compression_algorithms.push_back("FS");	  //Frequent subtrajectories
	
//	Parameters::set_compression_algorithms(compression_algorithms);
//	unsigned int num_updates;	
//	TrajCompAlgo* alg;

	/*Reading the input parameters*/
//	if(Parameters::read(argc,argv))
//	{
//		Parameters::print();

		/*Reading road network*/
//		RoadNet* net = new RoadNet(Parameters::road_net_file_name);
		
//		if(Parameters::compression_algorithm == "FS")
//		{
//			alg = new FreqSubt(Parameters::min_sup, net);
//		}

//		alg->train(Parameters::training_traj_file_name);

//		num_updates = alg->test(Parameters::test_traj_file_name);

//		std::cout << "num updates = " << num_updates << std::endl;
		
//		delete alg;
//		delete net;
//	}

	return 0;
}

