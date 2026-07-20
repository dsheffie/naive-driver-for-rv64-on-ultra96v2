OBJ = axi.o helper.o saveState.o loadelf.o disassemble.o driver.o command_parsing.o mon.o
CXX = g++
EXE = mips-axi
OPT = -O3 -g
CXXFLAGS = -std=c++11 -DFAITHFUL_SCSI -g $(OPT)
DEP = $(OBJ:.o=.d)
LIBS = -lboost_program_options -lcapstone
.PHONY: all clean

all: $(EXE) mipsmon

$(EXE) : $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) $(LIBS) -o $(EXE)

mipsmon: mipsmon.cc
	$(CXX) $(CXXFLAGS) mipsmon.cc -o mipsmon -lncurses

%.o: %.cc
	$(CXX) -MMD $(CXXFLAGS) -c $< 

-include $(DEP)

clean:
	rm -rf $(EXE) mipsmon $(OBJ) $(DEP)
