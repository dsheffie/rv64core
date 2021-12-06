#include "helper.hh"
#include "interpret.hh"
#include "disassemble.hh"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <array>
#include <map>
#include <string>
#include <capstone/capstone.h>

#include "disassemble.hh"

static const std::array<std::string,32> regNames = {
  "zero","at", "v0", "v1","a0", "a1", "a2", "a3",
    "t0", "t1", "t2", "t3","t4", "t5", "t6", "t7",
    "s0", "s1", "s2", "s3","s4", "s5", "s6", "s7",
    "t8", "t9", "k0", "k1","gp", "sp", "s8", "ra"
    };

static const std::array<std::string,16> condNames = {
  "f", "un", "eq", "ueq", "olt", "ult", "ole", "ule",
  "sf", "ngle", "seq", "ngl", "lt", "nge", "le", "ngt"
};

const std::string &getCondName(uint32_t c) {
  return condNames[c&15];
}

const std::string &getGPRName(uint32_t r) {
  return regNames[r&31];
}



void disassemble(std::ostream &out, uint32_t inst, uint32_t addr) {
  out << getAsmString(inst,addr);
}


static const std::map<cs_err, std::string> cs_error_map =
  {
   {CS_ERR_OK,"CS_ERR_OK"},
   {CS_ERR_MEM,"CS_ERR_MEM"},
   {CS_ERR_ARCH,"CS_ERR_ARCH"},
   {CS_ERR_HANDLE,"CS_ERR_HANDLE"},
   {CS_ERR_CSH,"CS_ERR_CSH"},
   {CS_ERR_MODE,"CS_ERR_MODE"},
   {CS_ERR_DETAIL,"CS_ERR_DETAIL"},
   {CS_ERR_MEMSETUP,"CS_ERR_MEMSETUP"},
   {CS_ERR_VERSION,"CS_ERR_VERSION"},
   {CS_ERR_DIET,"CS_ERR_DIET"},
   {CS_ERR_SKIPDATA,"CS_ERR_SKIPDATA"},
   {CS_ERR_X86_ATT,"CS_ERR_X86_ATT"},
   {CS_ERR_X86_INTEL,"CS_ERR_X86_INTEL"},               
  };

static csh handle;

void initCapstone() {
  cs_err C = cs_open(CS_ARCH_MIPS, CS_MODE_MIPS32, &handle);
  if(C != CS_ERR_OK) {
    std::cerr << "capstone error : " << cs_error_map.at(C) << "\n";
    exit(-1);
  }
}

void stopCapstone() {
  cs_close(&handle);
}

std::string getAsmString(uint32_t inst, uint32_t addr) {
  std::stringstream ss;

  cs_insn *insn = nullptr;

  size_t count = cs_disasm(handle,reinterpret_cast<const uint8_t *>(&inst),
			   sizeof(inst), addr, 0, &insn);
  if(count != 1) {
    return std::string();
  }
  //if(count != 1) {
  //std::cerr << "decoded count of " << count << " mips instructions?\n";
  //}
  //assert(count == 1);
  ss << insn[0].mnemonic << " " << insn[0].op_str;
  cs_free(insn, count);
  return ss.str();
}



