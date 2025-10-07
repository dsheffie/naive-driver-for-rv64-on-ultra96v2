OBJ = axi.o helper.o saveState.o
CXX = g++
EXE = axi
OPT = -O0 -g -I/usr/local/include/SDL2 -D_GNU_SOURCE=1 -D_REENTRANT
CXXFLAGS = -std=c++11 -g $(OPT)
DEP = $(OBJ:.o=.d)
LIBS = -lboost_program_options -L/usr/local/lib -Wl,-rpath,/usr/local/lib -lSDL2 -lpthread
.PHONY: all clean

all: $(EXE)

$(EXE) : $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) $(LIBS) -o $(EXE)

%.o: %.cc
	$(CXX) -MMD $(CXXFLAGS) -c $< 

-include $(DEP)

clean:
	rm -rf $(EXE) $(OBJ) $(DEP)
