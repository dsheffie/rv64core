# rv32core

Pedagogical SystemVerilog model for a decoupled RV32m CPU. Now that the design supports RV instead of MIPSII, it's gotta be approximately 3x times better (riscV - mipsII = 3x)

Relatively simple uarch - two-wide renamed, decoupled microarchitecture. 1 mem FU, 2 int FU.  Each int FU backed by matrix-style scheduler.

Caches - direct mapped. Keep things simple and actually implementable on FPGA. Make'em really big because you can do that these days. Never spent time on any elaborate forwarding schemes because it killed cycle time on my FPGA. 

Fetch - upto 4 instructions per cycle from the I$. Branch targets decoded directly from the I$ and correctly predicted direct branches have no penality.  64 entry BTB for indirect branches. 16 entry RSB for indirects.  Sizes configurable in the "machine.vh" configuration header. Instructions pushed into queue for decode.

Decode - straightforward implementation. uops placed in allocation queue.

Allocate - Check for sufficient resources (ROB entry, physical registers, space in the appropriate FU queue). "Freelist" implemented with a bitvector and banked to support two allocations per cycle (M1 explainer implies Apple does something similar - clearly I'm a microarchitectural genius). Some instructions  are folded and immediately mark complete after allocation (e.g., direct jumps or nops), they do not take execution slots. 

Schedule - Matrix-style schedulers pick oldest ready first. Single cycle schedule to execute loop. I haven't gotten around to figuring out replay schemes. Maybe someday.  

Execute - Nothing exciting.  Fully bypassed RF. 

Complete - Write back results to PRF.  Completion status to the ROB.

Graduate - Check for faults. If fault, rollback to state in retirement RAT.  Bitvectors make flash restart snappy.  
