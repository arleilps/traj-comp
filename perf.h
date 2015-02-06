/**
Copyright (c) 2011, Arlei Silva
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

@author: Arlei Silva (arleilps@gmail.com)
**/

#ifndef PERF_H
#define PERF_H

/*std includes*/
#include <utility>
#include <ctime>
#include <pthread.h>
#include <sys/times.h>
#include <sys/time.h>
#include <sys/resource.h>

/**
 *	Simple class for computing execution time of sections of code. 
**/

class ExecTime
{
	public:
		/**
		  * Starts the counter
		  * @param
		  * @return
		  * @throws
		 **/
		ExecTime()
		{
		    time = 0;
		}
		
		/**
		  * Destructor
		  * @param
		  * @return
		  * @throws
		 **/
		virtual ~ExecTime(){};

		/**
		  * Starts the counter
		  * @param
		  * @return
		  * @throws
		 **/
		inline void start()
		{
			gettimeofday (&start_time, NULL);
		}
		
		/**
		  * Resets the counter
		  * @param
		  * @return
		  * @throws
		 **/
		inline void reset()
		{
			time = 0;
		}
		
		
		/**
		  * Stops the counter
		  * @param
		  * @return
		  * @throws
		 **/
		inline void stop()
		{
			gettimeofday (&end_time, NULL);
			double tmp = ((double) end_time.tv_sec + (double) end_time.tv_usec / 1000000) - ((double) start_time.tv_sec + (double) start_time.tv_usec / 1000000);

			if(tmp > 0)
			{
				time += tmp;
			}
		}
		
		/**
		  * Returns the time in seconds
		  * @param
		  * @return
		  * @throws
		 **/
		const inline double get_seconds() const
		{
			return time;
		}

	private:
	        double time;
		struct timeval start_time;
		struct timeval end_time;

};


#endif
