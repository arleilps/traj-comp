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
	RoadNet* net = new RoadNet("../data/greater_sfo_adj.txt");
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
	
	std::cout << "Map matching" << std::endl;
	//Segment: 82654
	double latit_one = 37.737565;
	double latit_two = 37.73742;
	double longit_one = -122.450356;
	double longit_two = -122.450546;

	unsigned int seg = net->match_segment(latit_one, latit_two, 
		longit_one, longit_two);
	
	std::cout << "Map matching segment from (" << latit_one << 
		"," << longit_one << ") to (" << latit_two << 
		"," << longit_two << ") is " << net->seg_name(seg) <<
		std::endl;
	
	//Segment: 82655
	latit = 37.737304;
	longit = -122.450736;
	
	unsigned int new_seg = net->match_segment(latit, longit, seg, 100);
	
	std::cout << "Map matching segment from previous segment" << net->seg_name(seg) << 
		" to (" << latit << 
		"," << longit << ") is " << net->seg_name(new_seg) <<
		std::endl;

	/*Deleting road network*/
	std::cout << "Deleting a road network" << std::endl;
	delete net;
	std::cout << "Road network deleted" << std::endl;
	
	std::cout << "Test finished" << std::endl;

	return true;
}

