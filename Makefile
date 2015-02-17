CFLAGS = -Wall -g -Ofast -pthread #-pg
RM = rm -f
GP_OBJS = main.o io.o test_traj_comp.o test_moving_obj.o test_road_net.o moving_obj.o road_net.o traj_comp.o perf.o getopt_pp.o
CC = g++ $(CFLAGS)

all: traj_comp

traj_comp: $(GP_OBJS)
	@echo ""
	@echo " --- traj_comp ---"
	@$(CC) $(GP_OBJS) -o traj_comp -L/usr/local/lib -lpqxx -lpq -I/usr/local/include/pqxx 
	@echo ""

%.o: %.cc %.h
	@echo " --- COMPILING OBJECT \"$@\""
	@$(CC) $(INCLUDES) $< -c 

clean:
	$(RM) traj_comp *.o
	clear

