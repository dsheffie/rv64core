#ifdef __linux__

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
#include <sys/uio.h>
#include <sys/utsname.h>

#include <linux/utsname.h>

#include "interpret.hh"
#include "disassemble.hh"
#include "helper.hh"
#include "globals.hh"

/* stolen from QEMU */

#define COPY_UTSNAME_FIELD(dest, src) \
  do { \
    memcpy((dest), (src), std::min(sizeof(src), sizeof(dest)));	\
      (dest)[sizeof(dest) - 1] = '\0'; \
  } while (0)

int sys_uname(struct new_utsname *buf) {
  struct utsname uts_buf;
  if (uname(&uts_buf) < 0)
      return (-1);

#define UOS(x) {std::cerr << "offset of " << #x << " is " << offsetof(struct new_utsname, x) << " bytes\n"; }
  //UOS(sysname);
  //UOS(nodename);
  //UOS(release);
  //UOS(version);
  //UOS(machine);
  //UOS(domainname);
#undef UOS

  
  memset(buf, 0, sizeof(*buf));
  COPY_UTSNAME_FIELD(buf->sysname, uts_buf.sysname);
  COPY_UTSNAME_FIELD(buf->nodename, uts_buf.nodename);
  COPY_UTSNAME_FIELD(buf->release, uts_buf.release);
  COPY_UTSNAME_FIELD(buf->version, uts_buf.version);
  COPY_UTSNAME_FIELD(buf->machine, uts_buf.machine);
#ifdef _GNU_SOURCE
  COPY_UTSNAME_FIELD(buf->domainname, uts_buf.domainname);
#endif
  return (0);
}
#undef COPY_UTSNAME_FIELD
  
static void copy_to_target_stat (struct stat &st, uint8_t* ptr) {
  struct target_stat {
    unsigned	st_dev;
    int32_t	st_pad1[3];		/* Reserved for network id */
    uint32_t	st_ino;
    unsigned int	st_mode;
    unsigned int	st_nlink;
    int		st_uid;
    int		st_gid;
    unsigned 	st_rdev;
    int32_t	st_pad2[2];
    int32_t	st_size;
    int32_t	st_pad3;
    /*
     * Actually this should be timestruc_t st_atime, st_mtime and st_ctime
     * but we don't have it under Linux.
     */
    int32_t		target_st_atime;
    int32_t		target_st_atime_nsec;
    int32_t		target_st_mtime;
    int32_t		target_st_mtime_nsec;
    int32_t		target_st_ctime;
    int32_t		target_st_ctime_nsec;
    int32_t		st_blksize;
    int32_t		st_blocks;
    int32_t		st_pad4[14];
  } __attribute__((packed));
  target_stat *ts = reinterpret_cast<target_stat*>(ptr);
  ts->st_dev = bswap<false>(st.st_dev);
  ts->st_ino = bswap<false>(st.st_ino);
  ts->st_mode = bswap<false>(st.st_mode);
  ts->st_nlink = bswap<false>(st.st_nlink);
  ts->st_uid = bswap<false>(st.st_uid);
  ts->st_gid = bswap<false>(st.st_gid);
  ts->st_rdev = bswap<false>(st.st_rdev);
  ts->st_size = bswap<false>(st.st_size);  
}


void linux_o32_syscall(state_t *s){
  uint32_t id = s->gpr[R_v0] - 4000;
  std::cout << "trapped into syscall number " << id << "\n";
  switch(id)
    {
    case 3: { /* read */
      s->gpr[R_v0] = read(s->gpr[R_a0], s->mem[s->gpr[R_a1]], s->gpr[R_a2]);
      s->gpr[R_a3] = 0;
      return;
    }
    case 4: {/* write */
      s->gpr[R_v0] = write(s->gpr[R_a0], s->mem[s->gpr[R_a1]], s->gpr[R_a2]);
      s->gpr[R_a3] = 0;
      return;
    }
    case 6: { /* close */
      s->gpr[R_v0] = close(s->gpr[R_a0]);
      s->gpr[R_a3] = 0;
      return;
    }
    case 45: { /* brk */
      //std::cout << "brk argument = "
      //<< std::hex
      //		<< s->gpr[R_a0]
      //	<< std::dec
      //	<< "\n";
      
      if(s->gpr[R_a0] == 0) {
	s->gpr[R_v0] = s->linux_image.target_brk;
	return;
      }
      if(s->gpr[R_a0] < s->linux_image.target_original_brk) {
	s->gpr[R_v0] = s->linux_image.target_brk;
	return;
      }
      if(s->gpr[R_a0] <= s->linux_image.brk_page) {
	abort();
      }
      s->linux_image.target_brk = s->gpr[R_a0];
      uint32_t allocsz = s->gpr[R_a0] - s->linux_image.brk_page;
      allocsz = roundToPgSz(allocsz, s->linux_image.pgsz);
      //((allocsz + linux_pgsz - 1) / linux_pgsz) * linux_pgsz;
      s->mem.prefault(s->linux_image.brk_page, allocsz);
      s->linux_image.brk_page = roundToPgSz(s->linux_image.brk_page, s->linux_image.pgsz);
	//((brk_page + linux_pgsz - 1) / linux_pgsz) * linux_pgsz;
      s->gpr[R_v0] = s->linux_image.target_brk;
      s->gpr[R_a3] = 0;
      return;
    }
    case 85: { /* readlink */
      //ssize_t readlink(const char *pathname, char *buf, size_t bufsiz);a
      //std::cout << "pathname = " << reinterpret_cast<const char*>(s->mem[s->gpr[R_a0]]) << "\n";
      
      s->gpr[R_v0] = readlink(reinterpret_cast<const char*>(s->mem[s->gpr[R_a0]]),
			      reinterpret_cast<char*>(s->mem[s->gpr[R_a1]]),
			      s->gpr[R_a2]
			      );

      //std::cout << "buf = " << reinterpret_cast<const char*>(s->mem[s->gpr[R_a1]]) << "\n";
      
      s->gpr[R_a3] = 0;
      return;

    }
    case 122: { /* uname */
      uint64_t start_page = s->gpr[R_a0] / 4096;
      uint64_t end_page = (s->gpr[R_a0] + sizeof(struct new_utsname)) / 4096;
      assert(start_page == end_page);
      struct new_utsname * buf = reinterpret_cast<struct new_utsname*>(s->mem[s->gpr[R_a0]]);
      sys_uname(buf);
      strncpy(buf->machine, "mips", sizeof(buf->machine));
      //std::cout << "version = " << buf->version << "\n";
      //std::cout << "release = " << buf->release << "\n";
      s->gpr[R_v0] = 0;
      s->gpr[R_a3] = 0;
      return;
    }
    case 146: { /*ssize_t writev(int fd, const struct iovec *iov, int iovcnt); */
      iovec iov;
      //
      uint32_t ptr = __builtin_bswap32(*reinterpret_cast<uint32_t*>(s->mem[s->gpr[R_a1]]));
      iov.iov_base = s->mem[ptr];
      iov.iov_len =  __builtin_bswap32(*reinterpret_cast<uint32_t*>(s->mem[s->gpr[R_a1]+4]));
      s->gpr[R_v0] = writev(s->gpr[R_a0], &iov, s->gpr[R_a2]);
      s->gpr[R_a3] = 0;
      return;
    }
    case 215: { /* fstat64 */
      struct stat st;
      s->gpr[R_v0] = fstat(s->gpr[R_a0], &st);
      s->gpr[R_a3] = 0;
      copy_to_target_stat(st, s->mem[s->gpr[R_a1]]);
      return;
    }
    case 225: {
      abort();
    }
    case 246: { /* exit_group */
      s->brk = 1;
      return;
    }
    case 283: { /* set_thread_area - store in cp9 reg 29*/
      //std::cerr << "thread_area = " << std::hex << s->gpr[R_a0] << std::dec << "\n";
      s->cpr0[29] = s->gpr[R_a0];
      s->gpr[R_v0] = 0;
      s->gpr[R_a3] = 0;      
      return;
    }
    case 288: {/* int openat(int dirfd, const char *pathname, int flags, mode_t mode); */
      s->gpr[R_v0] = openat(s->gpr[R_a0],
			    reinterpret_cast<const char*>(s->mem[s->gpr[R_a1]]),
			    s->gpr[R_a2], s->gpr[R_a3]);
#if 0
      std::cerr << std::hex
		<< s->gpr[R_a0]
		<< ","
		<< s->gpr[R_a1]
		<< ","
		<< s->gpr[R_a2]
		<< ","
		<< s->gpr[R_a3]
		<< std::dec
		<< "\n";
      std::cout << "pathname = " << s->mem[s->gpr[R_a1]] << "\n";
#endif
      s->gpr[R_a3] = 0;
      return;
    }
    default:
      abort();
    }
}
#endif
