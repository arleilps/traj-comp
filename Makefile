SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic
CPLEXDIR      = /opt/ibm/ILOG/CPLEX_Studio_Community1262/cplex/
CONCERTDIR    = /opt/ibm/ILOG/CPLEX_Studio_Community1262/concert/
CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD
CPLEXBINDIR   = $(CPLEXDIR)/bin/$(BINDIST)
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include
CCLNDIRS  = -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR)
CCLNFLAGS = -lconcert -lilocplex -lcplex -lm -lpthread
CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR)

CFLAGS = -Wall -g -pthread
RM = rm -f
GP_OBJS = main.o io.o test_traj_comp.o test_moving_obj.o test_road_net.o moving_obj.o road_net.o traj_comp.o perf.o getopt_pp.o
CC = g++ $(CFLAGS) $(CCLNFLAGS)

all: traj_comp

traj_comp: $(GP_OBJS)
	@echo ""
	@echo " --- traj_comp ---"
	@$(CC) $(GP_OBJS) $(CCFLAGS) $(CCLNDIRS) -o traj_comp -L/usr/local/lib -lpqxx -lpq -I/usr/local/include/pqxx $(CCLNFLAGS)
	@echo ""

%.o: %.cc %.h
	@echo " --- COMPILING OBJECT \"$@\""
	@$(CC) $(INCLUDES) $(CCFLAGS) $(CCLNDIRS) $(CCLNFLAGS) $< -c 

clean:
	$(RM) traj_comp *.o
	clear

