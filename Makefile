OBJ = axi.o helper.o saveState.o
CXX = g++
EXE = axi
OPT = -O3
CXXFLAGS = -std=c++11 -g $(OPT)
DEP = $(OBJ:.o=.d)
LIBS = -lboost_program_options
.PHONY: all clean

all: $(EXE)

$(EXE) : $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) $(LIBS) -o $(EXE)

%.o: %.cc
	$(CXX) -MMD $(CXXFLAGS) -c $< 

-include $(DEP)

clean:
	rm -rf $(EXE) $(OBJ) $(DEP)
