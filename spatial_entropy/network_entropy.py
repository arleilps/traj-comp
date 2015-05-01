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
import networkx

def read_graph(input_file_name):
    input_file = open(input_file_name, 'r')
    g = networkx.DiGraph()

    num_vertices = int(input_file.readline())
    
    for i in range(0, num_vertices):
    	input_file.readline()

    for j in range(0, num_vertices):
    	line = input_file.readline()
    	vec = line.rsplit(',')

	for i in range(1, len(vec)):
	    g.add_edge(int(vec[0]), int(vec[i]))

    input_file.close()

    return g

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
	        print "python network_entropy.py [-i <input_file>]"
	        sys.exit()
 	
	g = read_graph(input_file_name)
    	pr = networkx.pagerank(g)
    	od = g.out_degree(g.nodes())

	sum_ratios = 0
	
	for v in pr:
	    if od[v] > 0:
	        sum_ratios = sum_ratios + float(pr[v]) / od[v]

	h = 1 - sum_ratios

	print "entropy = ", h

    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2

if __name__ == "__main__":
    sys.exit(main())
