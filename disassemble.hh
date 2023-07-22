#include <string>
#include <cstdint>

#ifndef __DISASSEMBLE_HH__
#define __DISASSEMBLE_HH__

const std::string &getCondName(uint32_t c);
const std::string &getGPRName(uint32_t r);

void initCapstone();
void stopCapstone();

std::string getAsmString(uint32_t inst,uint32_t addr);
void disassemble(std::ostream &out, uint32_t inst, uint32_t addr);


#endif
