#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cassert>
#include <utility>
#include <cstdint>
#include <list>
#include <map>

#include "helper.hh"
#include "interpret.hh"
#include "globals.hh"

#ifdef __APPLE__
#include "TargetConditionals.h"
#ifdef TARGET_OS_MAC
#include "osx_elf.h"
#endif
#else
#include <elf.h>
#endif

static const uint8_t magicArr[4] = {0x7f, 'E', 'L', 'F'};

static bool checkElf(const Elf64_Ehdr *eh) {
  const uint8_t *identArr = reinterpret_cast<const uint8_t*>(eh->e_ident);
  return memcmp((void*)magicArr, identArr, 4)==0;
}

static bool check64Bit(const Elf64_Ehdr *eh) {
  return (eh->e_ident[EI_CLASS] == ELFCLASS64);
}

static bool checkLittleEndian(const Elf64_Ehdr *eh) {
  return (eh->e_ident[EI_DATA] == ELFDATA2LSB);
}

bool is_rv64_elf(const char* fn) {
  struct stat s;
  int fd = -1, rc = -1;
  bool success = false;
  Elf64_Ehdr *eh = nullptr;
  fd = open(fn, O_RDONLY);
  if(fd<0) {
    printf("open() returned %d\n", fd);
    exit(-1);
  }
  rc = fstat(fd,&s);
  if(rc<0) {
    printf("fstat() returned %d\n", rc);
    exit(-1);
  }
  eh = (Elf64_Ehdr*)mmap(nullptr, s.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    
  if(not(checkElf(eh))) {
    goto done;
  }

  if(not(check64Bit(eh))) {
    goto done;
  }
  if(not(checkLittleEndian(eh))) {
    goto done;
  }
  if((eh->e_machine) != 243) {
    printf("Not a RISCV binary..goodbye got type %d\n", (eh->e_machine));
    goto done;
  }
  success = true;
 done:
  munmap(reinterpret_cast<void*>(eh), s.st_size);
  close(fd);
  return success;
}

bool load_elf(const char* fn, state_t *ms) {
  struct stat s;
  Elf64_Ehdr *eh = nullptr;
  Elf64_Phdr* ph = nullptr;
  Elf64_Shdr* sh = nullptr;
  int32_t e_phnum=-1,e_shnum=-1;
  size_t pgSize = getpagesize();
  int fd,rc;
  char *buf = nullptr;
  uint8_t *mem = ms->mem;
  /* symbol code shamelessly stolen from
   * elfloader.cc - need for to/from host
   * symbols */
  int32_t strtabidx = 0, symtabidx = 0;

  fd = open(fn, O_RDONLY);
  if(fd<0) {
    printf("open() returned %d\n", fd);
    exit(-1);
  }
  rc = fstat(fd,&s);
  if(rc<0) {
    printf("fstat() returned %d\n", rc);
    exit(-1);
  }
  buf = (char*)mmap(nullptr, s.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
  eh = (Elf64_Ehdr *)buf;
  close(fd);
    
  if(not(checkElf(eh))) {
    return false;
  }

  if(not(check64Bit(eh))) {
    return false;
  }
  if(not(checkLittleEndian(eh))) {
    return false;
  }

  if((eh->e_machine) != 243) {
    printf("Not a RISCV binary..goodbye got type %d\n", (eh->e_machine));
    return false;
  }

  e_phnum = (eh->e_phnum);
  ph = reinterpret_cast<Elf64_Phdr*>(buf + eh->e_phoff);
  e_shnum = eh->e_shnum;
  sh = reinterpret_cast<Elf64_Shdr*>(buf + eh->e_shoff);
  char* shstrtab = buf + sh[eh->e_shstrndx].sh_offset;
  ms->pc = eh->e_entry;

  
  /* Find instruction segments and copy to
   * the memory buffer */
  for(int32_t i = 0; i < e_phnum; i++, ph++) {
    auto p_memsz = (ph->p_memsz);
    auto p_offset = (ph->p_offset);
    auto p_filesz = (ph->p_filesz);
    auto p_type = ph->p_type;
    auto p_vaddr = ph->p_vaddr;

#if 0
    std::cout << "p_type = "
	      << std::hex
	      << p_type
	      << " size = "
	      << p_memsz
	      << std::dec << "\n";
#endif
    
    if(p_type == SHT_PROGBITS && p_memsz) {

      //std::cout << "copying progbits :  "
      //<< std::hex << p_vaddr
      //	<< " to "
      //	<< (p_vaddr + p_memsz)
      //	<< std::dec << "\n";
      
      memset(mem+p_vaddr, 0, sizeof(uint8_t)*p_memsz);
      memcpy(mem+p_vaddr, (uint8_t*)(buf + p_offset),
	     sizeof(uint8_t)*p_filesz);
    }
  }

  
  for(int32_t i = 0; i < e_shnum; i++, sh++) {
    //int32_t f = (sh->sh_flags);
    // if(f & SHF_EXECINSTR) {
    //   uint32_t addr = (sh->sh_addr);
    //   int32_t size = (sh->sh_size);

    //   bool pgAligned = ((addr & 4095) == 0);
    //   if(pgAligned) {
    // 	size = (size / pgSize) * pgSize;
    // 	void *mpaddr = (void*)(mem+addr);
    // 	rc = mprotect(mpaddr, size, PROT_READ);
    // 	if(rc != 0) {
    // 	  printf("mprotect rc = %d, error(%d) = %s\n", rc, 
    // 		 errno, strerror(errno));
    // 	}
    //   }
    // }
    if (sh->sh_type & SHT_NOBITS) {
      continue;
    }
    if (strcmp(shstrtab + sh->sh_name, ".strtab") == 0) {
      strtabidx = i;
    }
    if (strcmp(shstrtab + sh->sh_name, ".symtab") == 0) {
      symtabidx = i;
    }
  }
  /* this code is all basically from elfloader.cc */
  if(strtabidx && symtabidx) {
    sh = reinterpret_cast<Elf64_Shdr*>(buf + eh->e_shoff);    
    char* strtab = buf + sh[strtabidx].sh_offset;
    Elf64_Sym* sym = reinterpret_cast<Elf64_Sym*>(buf + sh[symtabidx].sh_offset);
    for(int32_t i = 0; i < (sh[symtabidx].sh_size / sizeof(Elf64_Sym)); i++) {
      globals::symtab[strtab + sym[i].st_name] = static_cast<uint32_t>(sym[i].st_value);
    }
  }
  munmap(buf, s.st_size);

  auto it0 = globals::symtab.find("tohost");
  auto it1 = globals::symtab.find("fromhost");
  if(it0 != globals::symtab.end()) {
    globals::tohost_addr = it0->second;
  }
  if(it1 != globals::symtab.end()) {
    globals::fromhost_addr = it1->second;
  }

#define WRITE_WORD(EA,WORD) { *reinterpret_cast<uint32_t*>(mem + EA) = WORD; }

  WRITE_WORD(0x1000, 0x00000297); //0
  WRITE_WORD(0x1004, 0x02028593); //1
  WRITE_WORD(0x1008, 0xf1402573); //2
  WRITE_WORD(0x100c, 0x0182b283); //3
  WRITE_WORD(0x1010, 0x00028067); //4
  WRITE_WORD(0x1014, 0);
  WRITE_WORD(0x1018, ms->pc);
  WRITE_WORD(0x101c, 0);

  ms->pc = 0x1000;
  return true;
}
