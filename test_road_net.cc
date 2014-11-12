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
 *	FILE test_road_net.cc: Test road network processing
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
#include "road_net.h"
#include "test_road_net.h"
#include "io.h"

const bool test_road_net()
{
	std::cout << "Testing the road network class" << std::endl;
	
	/*Creating a road network*/
	std::cout << "Creating a road network" << std::endl;
	//RoadNet* net = new RoadNet("../data/greater_sfo_adj.txt", "road_net.csv");
	RoadNet* net = new RoadNet("road_net.csv");
	std::cout << "Road network created" << std::endl;

	double latit;
	double longit;
	
	std::cout << "Within distance" << std::endl;

	std::list<unsigned int> segs;
	double dist = 0;
	latit = 37.7401044;
	longit = -122.4337842;
	
	unsigned int count = net->segments_within_distance(segs, latit, longit, dist);

	std::cout << count << " segments at distance " << dist << " from (lat=" << 
		latit << ",long=" << longit << "):" << std::endl;

	for(std::list<unsigned int>::iterator it = segs.begin();
		it != segs.end(); ++it)
	{
		std::cout << "id: " << net->seg_name(*it) << " (" << net->seg_latit_begin(*it) << "," << 
			net->seg_longit_begin(*it) << ") - (" << 
			net->seg_latit_begin(*it) << "," <<
			net->seg_longit_end(*it) << ")" << std::endl;
	}

	segs.clear();
	dist = 10;
	
	count = net->segments_within_distance(segs, latit, longit, dist);
	
	std::cout << count << " segments at distance " << dist << " from (lat=" << 
		latit << ",long=" << longit << "):" << std::endl;

	for(std::list<unsigned int>::iterator it = segs.begin();
		it != segs.end(); ++it)
	{
		std::cout << "id: " << net->seg_name(*it) << " (" << net->seg_latit_begin(*it) << "," << 
			net->seg_longit_begin(*it) << ") - (" << 
			net->seg_latit_begin(*it) << "," <<
			net->seg_longit_end(*it) << ")" << std::endl;
	}
	
	/*Deleting road network*/
	std::cout << "Deleting a road network" << std::endl;
	delete net;
	std::cout << "Road network deleted" << std::endl;
	
	std::cout << "Test finished" << std::endl;

	return true;
}

