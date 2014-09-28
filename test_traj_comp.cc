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
 *	FILE test_moving_obj.cc: Test moving object functionalities
**/


/*std includes*/
#include <string>
#include <exception>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>

/*my includes*/
#include "test_traj_comp.h"
#include "traj_comp.h"

const bool test_traj_comp_freq_subt()
{
	std::cout << "Testing frequent sub-trajectory based compression" << std::endl;
	std::cout << "Creating a road network" << std::endl;
	RoadNet* net = new RoadNet("../data/greater_sfo_adj.txt");
	std::cout << "Road network created" << std::endl;

	std::cout << "Training the compression model" << std::endl;
	TrajCompAlgo* alg = new  FreqSubt(0, net);
	alg->train("../data/small_cab_stream_sfo.txt");
	std::cout << "Training finished" << std::endl;

	std::cout << "Deleting road network";
	delete net;
	std::cout << "Road network deleted";
	
	std::cout << "Test finished" << std::endl;
	return true;
}

	
