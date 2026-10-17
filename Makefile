OBJ = axi.o helper.o saveState.o loadelf.o disassemble.o driver.o command_parsing.o mon.o newport_arm.o sgi_newport.o sgi_ps2.o
CXX = g++
EXE = mips-axi
OPT = -O3 -g
CXXFLAGS = -std=c++11 -g $(OPT) -pthread
DEP = $(OBJ:.o=.d)
LIBS = -lboost_program_options -lcapstone -pthread
# the Newport window (--newport-display) needs Xlib; without it mips-axi is headless only
ifneq ($(wildcard /usr/include/X11/Xlib.h),)
CXXFLAGS += -DNEWPORT_X11
LIBS += -lX11
endif
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
