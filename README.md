# mipscore
Pedagogical SystemVerilog model for a decoupled MIPS CPU. Aspires to be used in either a FPGA implementation of a SGI workstation or N64.

Implements most of user-level MIPS-2, some of MIPS-3, some of MIPS-4, and some of MIPS32/MIPS64

Only 32b code has been tested.  Correctness checking infrastructure only really supports 32b MIPS too.

Floating-point implementation is correctness challenged.  Correct implementation of floating-point algorithms tricker than previously believed ;)

Relatively simple uarch - scalar renamed, decoupled microarchitecture. 1 mem FU, 1 int FU, and FPU.  Each FU is backed by an in order FIFO scheduler. E.g., within
each FU, instructions are scheduled in order but only true dependences prevent execution across execution classes. 

Caches - direct mapped. Keep things simple and actually implementable on FPGA.

Fetch - one instruction per cycle from the I$. Branch targets decoded directly from the I$ and correctly predicted direct branches have no penality.  64 entry
BTB for indirect branches. 4 entry RSB for indirects.  Sizes configurable in the "machine.vh" configuration header. Instructions pushed into queue for decode.

Decode - straightforward implementation. uops placed in allocation queue.

Allocate - Check for sufficient resources (ROB entry, physical registers, space in the appropriate FU queue). Allocate a renamed register from the 
appropriate register pool, if required.  Read RAT to map logical registers to physical registers. Update RAT if uop writes a dest.  Some instructions 
are folded and immediately mark complete after allocation (e.g., direct jumps or nops), they do not take execution slots. Allocation RAT immediately updated.

Executation - 

Complete - 

Graduate - (yes, using R10k terminology)
