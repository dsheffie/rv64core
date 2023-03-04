#ifndef __sparse_mem_hh__
#define __sparse_mem_hh__

#include <cstdlib>
#include <cassert>
#include <cstring>
#include <cstdint>


#include <map>

#include "sim_bitvec.hh"
#include "helper.hh"

#ifndef unlikely
#define unlikely(x)    __builtin_expect(!!(x), 0)
#endif

class sparse_mem {
public:
  static const uint64_t pgsize = 4096;
private:
  static const uint64_t sz = 1UL<<32;  
  uint8_t *mem = nullptr;
public:
  sparse_mem();
  ~sparse_mem();
  void clear();

  
  void prefault(uint64_t addr, uint64_t alloc_size=pgsize) {}
  void coalesce(uint64_t addr, uint64_t sz) {}
#if 1
  uint8_t * operator[](uint64_t addr) {
    return &mem[addr];
  }
  uint8_t * operator+(uint64_t disp) {
    return (*this)[disp];
  }
#endif
  bool compare(const sparse_mem &other) {
    bool error = false;
    for(uint64_t b = 0; b < sz; ++b) {
      if(mem[b] != other.mem[b]) {
	error = true;
	std::cout << "byte " << std::hex << b
		  << " differs "
		  << static_cast<int>(mem[b])
		  << " vs "
		  << static_cast<int>(other.mem[b])
		  << std::dec
		  << "\n";
      }
    }
    return error;
  }
  uint8_t *get_raw_ptr(uint64_t byte_addr) {
    byte_addr &= ((1UL<<32) - 1);
    return mem+byte_addr;
  }
  template <typename T>
  T get(uint64_t byte_addr) {
    assert(byte_addr < 1UL<<32);
    return *reinterpret_cast<T*>(mem+byte_addr);
  }
  template<typename T>
  void set(uint64_t byte_addr, T v) {
    assert(byte_addr < 1UL<<32);
    *reinterpret_cast<T*>(mem+byte_addr) = v;
  }
  uint64_t bytes_allocated() const {
    return sz;
  }
  template <bool do_write>
  friend int per_page_rdwr(sparse_mem &mem, int fd, uint64_t offset, uint64_t nbytes) {
    if(do_write) {
      return write(fd, &(mem.mem[offset]), nbytes);
    }
    return read(fd, &(mem.mem[offset]), nbytes);
  }
};



#if 0
class sparse_mem {
public:
  static const uint64_t pgsize = 4096;
private:
  std::map<uint64_t, uint8_t*> smem;
  std::map<uint8_t*, uint64_t> allocations;
  uint8_t *add_page(uint64_t addr);
  uint8_t* get_page(uint64_t addr);

public:
  sparse_mem() {};
  ~sparse_mem();
  uint32_t crc32() const;
  void prefault(uint64_t addr, uint64_t alloc_size=pgsize);
  void coalesce(uint64_t addr, uint64_t sz);
  uint8_t & at(uint64_t addr) {
    uint64_t baddr = addr % pgsize;
    uint8_t *m = get_page(addr);
    assert(m != nullptr);
    return m[baddr];
  }
  uint8_t * operator[](uint64_t addr) {
    uint64_t baddr = addr % pgsize;
    uint8_t *m = get_page(addr);
    return &m[baddr];
  }
  template <typename T>
  T get(uint64_t byte_addr) {
    uint64_t baddr = byte_addr % pgsize;
    uint8_t *m = get_page(byte_addr);
    return *reinterpret_cast<T*>(m+baddr);
  }
  template <typename T>
  T load(uint64_t byte_addr) {
    return get<T>(byte_addr);
  }      
  template<typename T>
  void set(uint64_t byte_addr, T v) {
    uint64_t baddr = byte_addr % pgsize;
    uint8_t *m = get_page(byte_addr);
    *reinterpret_cast<T*>(m+baddr) = v;
  }
  template<typename T>
  void store(uint64_t byte_addr, T v) {
    set<T>(byte_addr, v);
  }
  uint32_t get32(uint64_t byte_addr)  {
    return get<uint64_t>(byte_addr);
  }
  uint8_t * operator+(uint64_t disp) {
    return (*this)[disp];
  }
  uint64_t count() const {
    return smem.size();
  }
  uint64_t bytes_allocated() const {
    return pgsize * count();
  }
  void clear();
};

template <bool do_write>
int per_page_rdwr(sparse_mem &mem, int fd, uint64_t offset, uint64_t nbytes) {
  uint64_t last_byte = (offset+nbytes);
  int acc = 0, rc = 0;

  if(do_write) {
    while(offset != last_byte) {
      uint64_t next_page = (offset & (~(sparse_mem::pgsize-1))) + sparse_mem::pgsize;
      next_page = std::min(next_page, static_cast<uint64_t>(last_byte));
      uint64_t disp = (next_page - offset);
      mem.prefault(offset);
      rc = write(fd, mem + offset, disp);
      if(rc == 0)
	return 0;
      if(rc>=0) {
	acc += rc;
      }
      else {
	acc = -1;
      }
      offset += disp;
    }
  }
  else {
    uint8_t *buf = new uint8_t[nbytes];
    acc = read(fd, buf, nbytes);
    for(int64_t i = 0; i < acc; i++) {
      mem.at(offset+i) = buf[i];
    }
    delete [] buf;
  }
  return acc;
}
#endif

#undef unlikely
#endif
