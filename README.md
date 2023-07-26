# mipscore

Pedagogical SystemVerilog model for a decoupled RV32m CPU. Now that the design supports RV instead of MIPS2, it's gotta be approximately 3x times better (riscv - mips2 = 3x)

Relatively simple uarch - two-wide renamed, decoupled microarchitecture. 1 mem FU, 1 int FU.  Int FU backed by matrix-style scheduler.

Caches - direct mapped. Keep things simple and actually implementable on FPGA.

Fetch - up-to 4 instructions per cycle from the I$. Branch targets decoded directly from the I$ and correctly predicted direct branches have no penality.  64 entry
BTB for indirect branches. 4 entry RSB for indirects.  Sizes configurable in the "machine.vh" configuration header. Instructions pushed into queue for decode.

Decode - straightforward implementation. uops placed in allocation queue.

Allocate - Check for sufficient resources (ROB entry, physical registers, space in the appropriate FU queue). "Freelist" implemented with a bitvector and banked to support
two allocations per cycle. Some instructions  are folded and immediately mark complete after allocation (e.g., direct jumps or nops), they do not take execution slots. 

Schedule -

Execute - 

Complete - 

Graduate - (yes, using R10k terminology)
