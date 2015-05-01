"""
Copyright (c) 2014, Arlei Silva
All rights reserved.
      
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
	  
Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
	        
@author: Arlei Silva (arleilps@gmail.com)
"""

import sys
import getopt
import math

def read_trajectories(input_file_name):
    input_file = open(input_file_name, 'r')
    trajectories = []
    drivers = {}
    ID = 0

    for line in input_file:
        line = line.rstrip()
	vec = line.rsplit(',')
	driver = vec[0]

	if driver not in drivers:
	    drivers[driver] = ID
	    ID = ID + 1
	    trajectories.append([])

	for i in range(1, len(vec)-2):
	    seg = vec[i]
            trajectories[drivers[driver]].append(seg)

    input_file.close()

    return trajectories

def entropy_order(T, order):
    psi = {}

    for t in T:
        for i in range(1, len(t)):
	    if i >= order:
		str_rep = '-'.join(t[i-order:i])

		if str_rep not in psi:
		    psi[str_rep] = {}

		if t[i] not in psi[str_rep]:
		    psi[str_rep][t[i]] = 1
		else:
		    psi[str_rep][t[i]] = psi[str_rep][t[i]] + 1

    sum_traj = 0
    sum_max = 0
    
    for t in psi:
        max_j = 0 
        for j in psi[t]:
	    sum_traj = sum_traj + psi[t][j]
	    if psi[t][j] > max_j:
	        max_j = psi[t][j]
	sum_max = sum_max + max_j

    return 1 - float(sum_max) / sum_traj, len(psi)

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg
	    
def main(argv=None):
    if argv is None:
        argv = sys.argv

    #	Parameters:
    #		- input network	i
    #

    try:
        try:
            opts, input_files = getopt.getopt(argv[1:], "i:h", ["input=","help"])
        except getopt.error, msg:
            raise Usage(msg)
 
        input_file_name = ""

        for opt,arg in opts:
	    if opt in ('-i', '--input'):
	        input_file_name = arg
	    if opt in ('-h', '--help'):
	        print "python traj_entropy.py [-i <input_file>]"
	        sys.exit()
 	
	T = read_trajectories(input_file_name)
        orders = [1, 2, 3, 4, 5, 6]

        for order in orders:
	    (entropy, sk) = entropy_order(T, order)
    	    print order, " ", entropy

    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2

if __name__ == "__main__":
    sys.exit(main())
