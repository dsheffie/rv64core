#include "svdpi.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

static uint8_t *mem = NULL;

static const uint64_t MAGICNUM = 0x64646464beefd00dUL;

typedef struct {
  uint32_t va;
  uint8_t data[4096];
} __attribute__((packed)) page;

typedef struct {
  uint64_t magic;
  uint64_t pc;
  int64_t gpr[32];
  uint64_t icnt;
  uint32_t num_nz_pages;
  uint64_t tohost_addr;
  uint64_t fromhost_addr;  
} __attribute__((packed)) header;

DPI_DLLESPEC
void load_mem() {
  const char filename[] = "bbl.bin0.bin";
  header h;
  page p;
  mem = (uint8_t*)malloc(1UL<<32);
  int fd = open(filename, O_RDONLY, 0600);
  assert(fd != -1);
  size_t sz = read(fd, &h, sizeof(h));
  assert(sz == sizeof(h));
  for(uint32_t i = 0; i < h.num_nz_pages; i++) {
    sz = read(fd, &p, sizeof(p));
    assert(sz == sizeof(p));
    memcpy(mem+p.va, p.data, 4096);
  }
  printf("memory has been loaded\n");
  close(fd);
}


DPI_DLLESPEC
long long read_mem64(long long addr) {
  return *(long long*)(mem+addr);
}

DPI_DLLESPEC
void write_mem64(long long addr, long long data) {
  *(long long*)(mem+addr) = data;
}
