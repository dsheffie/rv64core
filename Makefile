UNAME_S = $(shell uname -s)

OBJ = top.o verilated.o verilated_vcd_c.o loadelf.o interpret.o disassemble.o helper.o saveState.o syscall.o temu_code.o

SV_SRC = core_l1d_l1i.sv core.sv exec.sv decode_riscv.sv shiftregbit.sv shift_right.sv mul.sv find_first_set.sv divider.sv l1d.sv l1i.sv machine.vh predecode.sv rob.vh uop.vh ram1r1w.sv ram2r1w.sv popcount.sv count_leading_zeros.sv fair_sched.sv csa.sv rf6r3w.sv reg_ram1rw.sv perfect_l1d.sv perfect_l1i.sv l2.sv l2_2way.sv mwidth_add.sv addsub.sv tlb.sv mmu.sv l1i_2way.sv nu_l1d.sv compute_pht_idx.sv

ifeq ($(UNAME_S),Linux)
	CXX = clang++-14 -flto
	MAKE = make
	VERILATOR_SRC = /home/dsheffie/local/share/verilator/include/verilated.cpp
	VERILATOR_VCD = /home/dsheffie/local/share/verilator/include/verilated_vcd_c.cpp
	VERILATOR_INC = /home/dsheffie/local/share/verilator/include
	VERILATOR_DPI_INC = /home/dsheffie/local/share/verilator/include/vltstd/
	VERILATOR = /home/dsheffie/local/bin/verilator
	EXTRA_LD = -lcapstone -lboost_program_options  -lboost_serialization -lunwind
endif

ifeq ($(UNAME_S),FreeBSD)
	CXX = CC -march=native
	VERILATOR_SRC = /opt/local/share/verilator/include/verilated.cpp
	VERILATOR_INC = /opt/local/share/verilator/include
	VERILATOR_VCD = /opt/local/share/verilator/include/verilated_vcd_c.cpp
        EXTRA_LD = -L/usr/local/lib -lcapstone -lboost_program_options  -lboost_serialization
	MAKE = gmake
endif

ifeq ($(UNAME_S),Darwin)
	CXX = clang++ -I/opt/local/include -flto
	VERILATOR_SRC = /Users/dsheffie/local/share/verilator/include/verilated.cpp
	VERILATOR_INC = /Users/dsheffie/local/share/verilator/include
	VERILATOR_VCD = /Users/dsheffie/local/share/verilator/include/verilated_vcd_c.cpp
	VERILATOR_DPI_INC = /Users/dsheffie/local/share/verilator/include/vltstd/
	VERILATOR = /Users/dsheffie/local/bin/verilator
	EXTRA_LD = -L/opt/local/lib -lboost_program_options-mt -lboost_serialization-mt -lcapstone
endif

OPT = -O3 -g -std=c++14 -fomit-frame-pointer
CXXFLAGS = -std=c++11 -g  $(OPT) -I$(VERILATOR_INC) -I$(VERILATOR_DPI_INC) #-DLINUX_SYSCALL_EMULATION=1
LIBS =  $(EXTRA_LD) -lpthread

DEP = $(OBJ:.o=.d)

EXE = rv64_core

.PHONY : all clean

all: $(EXE)

$(EXE) : $(OBJ) obj_dir/Vcore_l1d_l1i__ALL.a
	$(CXX) $(CXXFLAGS) $(OBJ) obj_dir/*.o $(LIBS) -o $(EXE)

top.o: top.cc obj_dir/Vcore_l1d_l1i__ALL.a
	$(CXX) -MMD $(CXXFLAGS) -Iobj_dir -c $< 

verilated.o: $(VERILATOR_SRC)
	$(CXX) -MMD $(CXXFLAGS) -c $< 

verilated_vcd_c.o: $(VERILATOR_VCD)
	$(CXX) -MMD $(CXXFLAGS) -c $< 

%.o: %.cc
	$(CXX) -MMD $(CXXFLAGS) -c $< 

obj_dir/Vcore_l1d_l1i__ALL.a : $(SV_SRC)
	$(VERILATOR) --x-assign unique -cc core_l1d_l1i.sv
	$(MAKE) OPT_FAST="-O3 -flto" -C obj_dir -f Vcore_l1d_l1i.mk

gen_html : gen_html.cc pipeline_record.hh
	$(CXX) -MMD $(CXXFLAGS) gen_html.cc $(LIBS) -o gen_html

pipe_to_txt : pipe_to_txt.cc pipeline_record.hh
	$(CXX) -MMD $(CXXFLAGS) pipe_to_txt.cc $(LIBS) -o pipe_to_txt


-include $(DEP)



clean:
	rm -rf $(EXE) $(OBJ) $(DEP) obj_dir
