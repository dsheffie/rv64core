#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <map>
#include <stack>

#include "interpret.hh"
#include "disassemble.hh"
#include "helper.hh"
#include "globals.hh"

static std::map<state_t*, std::map<int, int>> fdmap;
static std::map<state_t*, int> fdcnt;


void handle_syscall(state_t *s, uint64_t tohost) {
  uint8_t *mem = s->mem;  
  if(tohost & 1) {
    /* exit */
    s->brk = 1;
    return;
  }
  uint64_t *buf = reinterpret_cast<uint64_t*>(mem + tohost);
  std::map<int, int> &m = fdmap[s];
   
  switch(buf[0])
    {
    case SYS_write: {/* int write(int file, char *ptr, int len) */
      int fd = (buf[1] > 2) ? m.at(buf[1]) : buf[1];
      buf[0] = write(fd, (void*)(s->mem + buf[2]), buf[3]);
      if(fd==1)
	fflush(stdout);
      else if(fd==2)
	fflush(stderr);
      break;
    }
    case SYS_open: {
      const char *path = reinterpret_cast<const char*>(s->mem + buf[1]);
      int fd = open(path, remapIOFlags(buf[2]), S_IRUSR|S_IWUSR);
      if(fd > 0) {
	int fd_ = static_cast<int>(fdcnt[s]) + 3;
	//printf("pointer %p : real fd = %d, remapped fd = %d\n", s, fd, fd_);      	
	m[fd_] = fd;      
	buf[0] = fd_;
	fdcnt[s]++;
      }
      else {
	buf[0] = fd;
      }
      break;
    }
    case SYS_close: {
      buf[0] = 0;
      if(buf[1] > 2) {
	buf[0] = close(m.at(buf[1]));
	m.erase(buf[1]);
      }
      break;
    }
    case SYS_read: {
      int fd = (buf[1] > 2) ? m.at(buf[1]) : buf[1];
      //printf("buf[1] = %d, attempting to read fd %d\n", buf[1], fd);
      buf[0] = read(fd, reinterpret_cast<char*>(s->mem + buf[2]), buf[3]); 
      break;
    }
    case SYS_lseek: {
      int fd = (buf[1] > 2) ? m.at(buf[1]) : buf[1];            
      buf[0] = lseek(fd, buf[2], buf[3]);
      break;
    }
    case SYS_fstat : {
      struct stat native_stat;
      int fd = (buf[1] > 2) ? m.at(buf[1]) : buf[1];            
      int rc = fstat(fd, &native_stat);
      buf[0] = rc;
      break;
    }
    case SYS_stat : {
      buf[0] = 0;
      break;
    }
    case SYS_gettimeofday: {
      static_assert(sizeof(struct timeval)==16, "struct timeval size");
      struct timeval *tp = reinterpret_cast<struct timeval*>(s->mem + buf[1]);
      struct timezone *tzp = reinterpret_cast<struct timezone*>(s->mem + buf[2]);
      buf[0] = gettimeofday(tp, tzp);
      break;
    }
    case SYS_times: {
      struct tms32 {
	uint32_t tms_utime;
	uint32_t tms_stime;  /* system time */
	uint32_t tms_cutime; /* user time of children */
	uint32_t tms_cstime; /* system time of children */
      };
      tms32 *t = reinterpret_cast<tms32*>(s->mem + buf[1]);
      struct tms tt;
      buf[0] = times(&tt);
      t->tms_utime = tt.tms_utime;
      t->tms_stime = tt.tms_stime;
      t->tms_cutime = tt.tms_cutime;
      t->tms_cstime = tt.tms_cstime;
      break;
    }
    default:
      std::cout << "syscall " << buf[0] << " unsupported\n";
      std::cout << *s << "\n";
      exit(-1);
    }
  //ack
  *reinterpret_cast<uint64_t*>(mem + globals::tohost_addr) = 0;
  *reinterpret_cast<uint64_t*>(mem + globals::fromhost_addr) = 1;
}
